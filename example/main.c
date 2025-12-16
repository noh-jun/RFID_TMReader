#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "mercuryapi.h"

/* ===========================
 * terminal (ESC to stop)
 * =========================== */
static struct termios g_orig_termios;

static void term_restore(void)
{
  tcsetattr(STDIN_FILENO, TCSANOW, &g_orig_termios);
}

static void term_raw(void)
{
  struct termios raw;
  if (tcgetattr(STDIN_FILENO, &g_orig_termios) != 0)
  {
    perror("tcgetattr");
    return;
  }

  atexit(term_restore);

  raw = g_orig_termios;
  raw.c_lflag &= ~(ICANON | ECHO);
  raw.c_cc[VMIN] = 0;
  raw.c_cc[VTIME] = 0;

  if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0)
  {
    perror("tcsetattr");
    return;
  }

  int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
  if (flags >= 0)
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
}

static int esc_pressed(void)
{
  unsigned char c = 0;
  ssize_t n = read(STDIN_FILENO, &c, 1);
  if (n == 1 && c == 27) /* ESC */
    return 1;
  return 0;
}

/* ===========================
 * log helpers
 * =========================== */
static const char* rc_to_str(mercuryapi_result_t rc)
{
  switch (rc)
  {
    case MERCURYAPI_OK: return "OK";
    case MERCURYAPI_ERR: return "ERR";
    case MERCURYAPI_ERR_INVALID_ARG: return "INVALID_ARG";
    case MERCURYAPI_ERR_NOT_CONNECTED: return "NOT_CONNECTED";
    case MERCURYAPI_ERR_IO: return "IO";
    case MERCURYAPI_ERR_NO_TAG: return "NO_TAG";
    default: return "UNKNOWN";
  }
}

/* yyyy-mm-dd HH:MM:SS.mmm */
static void format_now(char* out, size_t out_size)
{
  if (!out || out_size == 0) return;

  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);

  struct tm tm_local;
  localtime_r(&ts.tv_sec, &tm_local);

  char buf[64];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm_local);

  long msec = ts.tv_nsec / 1000000L;
  snprintf(out, out_size, "%s.%03ld", buf, msec);
}

int main(int argc, char** argv)
{
  printf("=============================================\n");
  printf(" RFID Test App (MercuryApi wrapper)\n");
  printf(" ESC to stop\n");
  printf(" Output: time, antenna, tag_count, tag_values...\n");
  printf("=============================================\n");

  if (argc != 2)
  {
    fprintf(stderr, "Usage: %s tmr:///dev/ttyUSB0\n", argv[0]);
    return 1;
  }

  const char* uri = argv[1];
  printf("[MAIN] uri=%s\n", uri);

  term_raw();

  mercuryapi_reader_t* reader = NULL;
  mercuryapi_result_t last_rc = MERCURYAPI_OK;

  /* ---- create ---- */
  printf("[CALL] mercuryapi_reader_create(&reader)\n");
  last_rc = mercuryapi_reader_create(&reader);
  printf("[RET ] mercuryapi_reader_create => %s(%d), reader=%p\n",
         rc_to_str(last_rc), (int)last_rc, (void*)reader);
  if (last_rc != MERCURYAPI_OK || !reader)
    return 1;

  /* ---- connect ---- */
  printf("[CALL] mercuryapi_reader_connect(reader, uri)\n");
  last_rc = mercuryapi_reader_connect(reader, uri);
  printf("[RET ] mercuryapi_reader_connect => %s(%d)\n",
         rc_to_str(last_rc), (int)last_rc);
  if (last_rc != MERCURYAPI_OK)
  {
    printf("[CALL] mercuryapi_reader_destroy(reader)\n");
    mercuryapi_reader_destroy(reader);
    return 1;
  }

  /* ---- region ---- */
  printf("[CALL] mercuryapi_reader_set_region_enum(reader, KR2)\n");
  last_rc = mercuryapi_reader_set_region_enum(reader, MERCURYAPI_REGION_KR2);
  printf("[RET ] mercuryapi_reader_set_region_enum => %s(%d)\n",
         rc_to_str(last_rc), (int)last_rc);
  if (last_rc != MERCURYAPI_OK)
  {
    printf("[CALL] mercuryapi_reader_disconnect(reader)\n");
    mercuryapi_reader_disconnect(reader);
    printf("[CALL] mercuryapi_reader_destroy(reader)\n");
    mercuryapi_reader_destroy(reader);
    return 1;
  }

  /* ---- antennas ---- */
  int ants[2] = {1, 2};
  printf("[CALL] mercuryapi_set_read_plan_gen2(reader, {1,2}, 2, -1)\n");
  last_rc = mercuryapi_set_read_plan_gen2(reader, ants, 2, -1);
  printf("[RET ] mercuryapi_set_read_plan_gen2 => %s(%d)\n",
         rc_to_str(last_rc), (int)last_rc);
  if (last_rc != MERCURYAPI_OK)
  {
    printf("[CALL] mercuryapi_reader_disconnect(reader)\n");
    mercuryapi_reader_disconnect(reader);
    printf("[CALL] mercuryapi_reader_destroy(reader)\n");
    mercuryapi_reader_destroy(reader);
    return 1;
  }

  printf("[MAIN] Connected. Reading loop start. (ESC to stop)\n");

  /* ---- read loop ---- */
  enum { MAX_TAGS_PER_READ = 64, EPC_BUF_SIZE = 256 };
  char epc_storage[MAX_TAGS_PER_READ][EPC_BUF_SIZE];
  char* epc_ptrs[MAX_TAGS_PER_READ];
  int epc_lens[MAX_TAGS_PER_READ];

  for (int t = 0; t < MAX_TAGS_PER_READ; ++t)
    epc_ptrs[t] = epc_storage[t];

  // /* 안테나별 식별 필요해서 1개씩 번갈아 plan 설정 */
  // last_rc = mercuryapi_set_read_plan_gen2(reader, ants, 2, -1);
  // if (last_rc != MERCURYAPI_OK)
  // {
  //   usleep(200 * 1000);
  // }

  while (!esc_pressed())
  {
    for (int i = 0; i < 1; ++i)
    {
      int ant = ants[i];

      /* 안테나별 식별 필요해서 1개씩 번갈아 plan 설정 */
      last_rc = mercuryapi_set_read_plan_gen2(reader, &ant, 1, -1);
      if (last_rc != MERCURYAPI_OK)
      {
        usleep(200 * 1000);
        continue;
      }

      /* 매번 버퍼 초기화 */
      for (int t = 0; t < MAX_TAGS_PER_READ; ++t)
      {
        memset(epc_storage[t], 0, EPC_BUF_SIZE);
        epc_lens[t] = EPC_BUF_SIZE; /* wrapper 구현이 "필요 크기"를 돌려줄 수 있으므로 넉넉히 */
      }

      int tag_count = 0;
      last_rc = mercuryapi_read_epcs(reader,
                                     epc_ptrs,
                                     epc_lens,
                                     MAX_TAGS_PER_READ,
                                     &tag_count,
                                     500 /* timeout_ms */);

      if (last_rc == MERCURYAPI_OK && tag_count > 0)
      {
        char ts[64];
        format_now(ts, sizeof(ts));

        /* 요구 포맷: time, antenna, tag count, tag values */
        printf("%s ant=%d count=%d", ts, ant, tag_count);
        for (int t = 0; t < tag_count; ++t)
        {
          /* epc_lens[t]는 "문자열 길이"로 오게 구현했으니 그대로 사용 가능 */
          epc_storage[t][EPC_BUF_SIZE - 1] = '\0';
          printf(" %s", epc_storage[t]);
        }
        printf("\n");
        fflush(stdout);
      }
      else if (last_rc == MERCURYAPI_ERR_NO_TAG)
      {
        /* tag 없음은 출력 안 함 */
      }
      else if (last_rc != MERCURYAPI_OK)
      {
        /* 에러도 출력 원치 않으면 이 블록도 제거 */
        fprintf(stderr, "[ERR ] ant=%d read failed: %s(%d)\n",
                ant, rc_to_str(last_rc), (int)last_rc);
      }
    }

    usleep(200 * 1000);
  }

  printf("\n[MAIN] ESC pressed. stopping...\n");

  /* ---- cleanup ---- */
  printf("[CALL] mercuryapi_reader_disconnect(reader)\n");
  last_rc = mercuryapi_reader_disconnect(reader);
  printf("[RET ] mercuryapi_reader_disconnect => %s(%d)\n",
         rc_to_str(last_rc), (int)last_rc);

  printf("[CALL] mercuryapi_reader_destroy(reader)\n");
  mercuryapi_reader_destroy(reader);
  printf("[RET ] mercuryapi_reader_destroy\n");

  return 0;
}
