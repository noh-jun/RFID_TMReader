#ifndef MERCURYAPI_H_
#define MERCURYAPI_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct mercuryapi_reader mercuryapi_reader_t;

typedef enum {
  MERCURYAPI_OK = 0,
  MERCURYAPI_ERR = -1,
  MERCURYAPI_ERR_INVALID_ARG = -2,
  MERCURYAPI_ERR_NOT_CONNECTED = -3,
  MERCURYAPI_ERR_IO = -4,
  MERCURYAPI_ERR_NO_TAG = -5
} mercuryapi_result_t;

// 생성/해제
mercuryapi_result_t mercuryapi_reader_create(mercuryapi_reader_t** out_reader);
void mercuryapi_reader_destroy(mercuryapi_reader_t* reader);

// 연결/해제
// uri 예: "tmr://192.168.1.100" / "tmr:///dev/ttyUSB0" (필요 시 포맷을 문서화)
mercuryapi_result_t mercuryapi_reader_connect(mercuryapi_reader_t* reader, const char* uri);
mercuryapi_result_t mercuryapi_reader_disconnect(mercuryapi_reader_t* reader);

/* ===== Region enum (limited) ===== */
typedef enum {
  MERCURYAPI_REGION_NA = 0,
  MERCURYAPI_REGION_EU = 1,
  MERCURYAPI_REGION_KR2 = 2
} mercuryapi_region_t;

/* Region 설정 (connect 이후 1회 호출 필수) */
mercuryapi_result_t mercuryapi_reader_set_region_enum(
    mercuryapi_reader_t* reader,
    mercuryapi_region_t region);

// 안테나 리스트 기반 ReadPlan 설정(GEN2)
mercuryapi_result_t mercuryapi_set_read_plan_gen2(
    mercuryapi_reader_t* reader,
    const int* antennas,
    int antenna_count,
    int read_power_dbm /* -1이면 변경 안 함 */);

// (선택) read + fetch를 분리하고 싶을 때 사용: 버퍼에서 EPC 1개 꺼내기
mercuryapi_result_t mercuryapi_fetch_one_epc(
    mercuryapi_reader_t* reader,
    char* out_epc,
    int* inout_len);

// 읽기 (최소 예: EPC 문자열만 반환)
// out_epc는 caller가 제공한 버퍼, inout_len은 입력=버퍼크기/출력=실제길이(널 제외)
mercuryapi_result_t mercuryapi_read_one_epc(
    mercuryapi_reader_t* reader,
    char* out_epc,
    int* inout_len,
    int timeout_ms);

mercuryapi_result_t mercuryapi_read_epcs(
    mercuryapi_reader_t* reader,
    char** epcs,          // [out] EPC 문자열 버퍼 포인터 배열
    int* epc_lens,        // [in/out] 각 버퍼 크기 입력, 실제 길이 출력
    int max_tags,         // 배열 최대 개수
    int* out_count,       // [out] 읽힌 태그 개수
    int timeout_ms);

// 버전
const char* mercuryapi_version_string(void);

#ifdef __cplusplus
}
#endif

#endif  // MERCURYAPI_H_
