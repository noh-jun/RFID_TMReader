#include "mercuryapi.h"

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>

#include "tm_reader.h"
#include "tm_config.h"
#include "tmr_read_plan.h"
#include "tmr_tag_protocol.h"
#include "tmr_tag_data.h"
#include "tm_reader.h"
#include "tmr_region.h"
#include "tmr_params.h"

struct mercuryapi_reader {
  TMR_Reader reader;
  int connected;
};

static void mercuryapi__log_tmr_error(TMR_Reader* r, TMR_Status st, const char* where)
{
  const char* err = TMR_strerr(r, st);
  if (!err) err = "(null)";
  fprintf(stderr,
          "[MercuryApi][%s] TMR_Status=%u, err=\"%s\"\n",
          where,
          (unsigned)st,
          err);
}

static mercuryapi_result_t map_status(TMR_Status st) {
  if (st == TMR_SUCCESS) return MERCURYAPI_OK;
  return MERCURYAPI_ERR;
}

static mercuryapi_result_t map_status_ex(TMR_Status st) {
  if (st == TMR_SUCCESS) return MERCURYAPI_OK;
  return MERCURYAPI_ERR_IO;
}

mercuryapi_result_t mercuryapi_reader_create(mercuryapi_reader_t** out_reader) {
  if (!out_reader) return MERCURYAPI_ERR_INVALID_ARG;

  mercuryapi_reader_t* r = (mercuryapi_reader_t*)calloc(1, sizeof(*r));
  if (!r) return MERCURYAPI_ERR;

  r->connected = 0;
  *out_reader = r;
  return MERCURYAPI_OK;
}

void mercuryapi_reader_destroy(mercuryapi_reader_t* reader) {
  if (!reader) return;
  if (reader->connected) {
    TMR_destroy(&reader->reader);
  }
  free(reader);
}

mercuryapi_result_t mercuryapi_reader_connect(mercuryapi_reader_t* reader, const char* uri) {
  if (!reader || !uri) return MERCURYAPI_ERR_INVALID_ARG;

  // init
  TMR_Status st = TMR_create(&reader->reader, uri);
  if (st != TMR_SUCCESS) return map_status(st);

  st = TMR_connect(&reader->reader);
  if (st != TMR_SUCCESS) {
    TMR_destroy(&reader->reader);
    return map_status(st);
  }

  reader->connected = 1;
  return MERCURYAPI_OK;
}

mercuryapi_result_t mercuryapi_reader_disconnect(mercuryapi_reader_t* reader) {
  if (!reader) return MERCURYAPI_ERR_INVALID_ARG;
  if (!reader->connected) return MERCURYAPI_ERR_NOT_CONNECTED;

  TMR_destroy(&reader->reader);
  reader->connected = 0;
  return MERCURYAPI_OK;
}

static int mercuryapi__map_region_enum(
    mercuryapi_region_t in,
    TMR_Region* out)
{
  if (!out) return 0;

  switch (in) {
    case MERCURYAPI_REGION_NA:
      *out = TMR_REGION_NA;
      return 1;

    case MERCURYAPI_REGION_EU:
      *out = TMR_REGION_EU;
      return 1;

    case MERCURYAPI_REGION_KR2:
      *out = TMR_REGION_KR2;
      return 1;

    default:
      return 0;
  }
}

mercuryapi_result_t mercuryapi_reader_set_region_enum(
    mercuryapi_reader_t* reader,
    mercuryapi_region_t region)
{
  if (!reader)
    return MERCURYAPI_ERR_INVALID_ARG;

  TMR_Region tmr_region;
  if (!mercuryapi__map_region_enum(region, &tmr_region))
    return MERCURYAPI_ERR_INVALID_ARG;

  TMR_Status st =
      TMR_paramSet(&reader->reader,
                   TMR_PARAM_REGION_ID,
                   &tmr_region);

  if (st != TMR_SUCCESS) {
    mercuryapi__log_tmr_error(&reader->reader, st, "TMR_paramSet(REGION_ID)");
    return MERCURYAPI_ERR_IO;
  }

  return MERCURYAPI_OK;
}

mercuryapi_result_t mercuryapi_set_read_plan_gen2(
    mercuryapi_reader_t* reader,
    const int* antennas,
    int antenna_count,
    int read_power_dbm) {
  if (!reader || !antennas || antenna_count <= 0) return MERCURYAPI_ERR_INVALID_ARG;
  if (!reader->connected) return MERCURYAPI_ERR_NOT_CONNECTED;

  if (antenna_count > 16) return MERCURYAPI_ERR_INVALID_ARG;

  uint8_t ant_u8[16];
  for (int i = 0; i < antenna_count; ++i) {
    if (antennas[i] <= 0 || antennas[i] > 255) return MERCURYAPI_ERR_INVALID_ARG;
    ant_u8[i] = (uint8_t)antennas[i];
  }

  TMR_ReadPlan plan;
  // 마지막 인자는 "weight" 또는 "readTime" 성격(버전에 따라 의미가 다름).
  // 단순 테스트는 1000 정도로 두면 무난.
  TMR_RP_init_simple(&plan, (uint8_t)antenna_count, ant_u8, TMR_TAG_PROTOCOL_GEN2, 1000);

  TMR_Status st = TMR_paramSet(&reader->reader, TMR_PARAM_READ_PLAN, &plan);
  if (st != TMR_SUCCESS) {
    mercuryapi__log_tmr_error(&reader->reader, st, "TMR_paramSet(REGION_ID)");
    return map_status_ex(st);
  }

  // read_power_dbm은 지금은 미적용(-1로 사용 권장).
  (void)read_power_dbm;

  return MERCURYAPI_OK;
}

mercuryapi_result_t mercuryapi_fetch_one_epc(
    mercuryapi_reader_t* reader,
    char* out_epc,
    int* inout_len) {
  if (!reader || !out_epc || !inout_len || *inout_len <= 0) return MERCURYAPI_ERR_INVALID_ARG;
  if (!reader->connected) return MERCURYAPI_ERR_NOT_CONNECTED;

  // 다음 태그가 있는지 확인
  TMR_Status st = TMR_hasMoreTags(&reader->reader);
  if (st != TMR_SUCCESS) return map_status_ex(st);

  TMR_TagReadData trd;
  st = TMR_getNextTag(&reader->reader, &trd);
  if (st != TMR_SUCCESS) return map_status_ex(st);

  int epc_len = (int)trd.tag.epcByteCount;
  const uint8_t* epc = (const uint8_t*)trd.tag.epc;

  int need = epc_len * 2 + 1;
  if (*inout_len < need) {
    *inout_len = need;
    return MERCURYAPI_ERR_INVALID_ARG;
  }

  static const char* kHex = "0123456789ABCDEF";
  for (int i = 0; i < epc_len; ++i) {
    out_epc[i * 2 + 0] = kHex[(epc[i] >> 4) & 0xF];
    out_epc[i * 2 + 1] = kHex[(epc[i] >> 0) & 0xF];
  }
  out_epc[epc_len * 2] = '\0';
  *inout_len = epc_len * 2;
  return MERCURYAPI_OK;
}

mercuryapi_result_t mercuryapi_read_one_epc(
    mercuryapi_reader_t* reader,
    char* out_epc,
    int* inout_len,
    int timeout_ms) {
  if (!reader || !out_epc || !inout_len || *inout_len <= 0) return MERCURYAPI_ERR_INVALID_ARG;
  if (!reader->connected) return MERCURYAPI_ERR_NOT_CONNECTED;

  int32_t tag_count = 0;
  TMR_Status st = TMR_read(&reader->reader, (uint32_t)timeout_ms, &tag_count);
  if (st != TMR_SUCCESS) {
    mercuryapi__log_tmr_error(&reader->reader, st, "TMR_read");
    return map_status_ex(st);
  }

  if (tag_count <= 0) {
    return MERCURYAPI_ERR_NO_TAG;
  }

  return mercuryapi_fetch_one_epc(reader, out_epc, inout_len);
}

mercuryapi_result_t mercuryapi_read_epcs(
    mercuryapi_reader_t* reader,
    char** epcs,
    int* epc_lens,
    int max_tags,
    int* out_count,
    int timeout_ms)
{
  if (!out_count) return MERCURYAPI_ERR_INVALID_ARG;
  *out_count = 0;

  if (!reader || !epcs || !epc_lens || max_tags <= 0)
    return MERCURYAPI_ERR_INVALID_ARG;
  if (!reader->connected)
    return MERCURYAPI_ERR_NOT_CONNECTED;

  int32_t tag_count = 0;
  TMR_Status st = TMR_read(&reader->reader, (uint32_t)timeout_ms, &tag_count);
  if (st != TMR_SUCCESS)
  {
    mercuryapi__log_tmr_error(&reader->reader, st, "TMR_read");
    return map_status_ex(st);
  }

  if (tag_count <= 0)
    return MERCURYAPI_ERR_NO_TAG;

  static const char* kHex = "0123456789ABCDEF";

  int n = 0;
  while (n < max_tags)
  {
    /* TMR_hasMoreTags는 "더 있으면 SUCCESS" 패턴으로 쓰는 예제가 많음 */
    st = TMR_hasMoreTags(&reader->reader);
    if (st != TMR_SUCCESS)
      break; /* 더 이상 없음(또는 내부적으로 더 못 가져옴) */

    TMR_TagReadData trd;
    st = TMR_getNextTag(&reader->reader, &trd);
    if (st != TMR_SUCCESS)
    {
      mercuryapi__log_tmr_error(&reader->reader, st, "TMR_getNextTag");
      return map_status_ex(st);
    }

    if (!epcs[n] || epc_lens[n] <= 0)
      return MERCURYAPI_ERR_INVALID_ARG;

    int epc_len = (int)trd.tag.epcByteCount;
    const uint8_t* epc = (const uint8_t*)trd.tag.epc;

    int need = epc_len * 2 + 1; /* 널 포함 */
    if (epc_lens[n] < need)
    {
      /* 호출자에게 필요한 버퍼 크기 알려주고 종료 */
      epc_lens[n] = need;
      *out_count = n;
      return MERCURYAPI_ERR_INVALID_ARG;
    }

    for (int i = 0; i < epc_len; ++i)
    {
      epcs[n][i * 2 + 0] = kHex[(epc[i] >> 4) & 0xF];
      epcs[n][i * 2 + 1] = kHex[(epc[i] >> 0) & 0xF];
    }
    epcs[n][epc_len * 2] = '\0';
    epc_lens[n] = epc_len * 2; /* 널 제외 실제 길이 */

    ++n;
  }

  *out_count = n;
  return (n > 0) ? MERCURYAPI_OK : MERCURYAPI_ERR_NO_TAG;
}
