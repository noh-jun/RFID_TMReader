// third_party/mercuryapi/api/rfid_api.c

#include "rfid_api.h"

#include <stdlib.h>   // malloc, free, qsort
#include <string.h>   // memset, strncpy
#include <stdint.h>

// MercuryAPI headers (CMake에서 third_party/mercuryapi/c/include 를 include dir로 추가하는 것을 전제)
#include "tm_reader.h"
#include "tmr_params.h"
#include "tmr_read_plan.h"
#include "tmr_region.h"
#include "tmr_tag_data.h"
#include "tmr_tag_protocol.h"
#include "tmr_status.h"

// 내부 상수
#define RFID_DEFAULT_PLAN_READTIME (1000U)
#define RFID_MAX_ANTENNAS        (16U)
#define RFID_REGIONLIST_MAX      (32U)

/**
 * @brief RFID Reader 상태를 관리하는 내부 컨텍스트 구조체.
 * @note 외부에는 opaque 타입(rfid_ctx_t)으로 노출되며, 구현부에서만 정의된다.
 *
 * @param reader      MercuryAPI Reader 핸들
 * @param reader_created Reader가 생성되었는지 여부(1: 생성됨, 0: 미생성)
 * @param initialized 초기화 상태(1: init 완료, 0: 미완료)
 * @param region      사용자가 지정한 RFID Region 값
 * @param readPowerDbm 설정된 읽기 전력(dBm), 0이면 기본값 사용
 */
typedef struct rfid_ctx {
    TMR_Reader reader;
    int reader_created;
    int initialized;
    RFID_REGION region;
    int readPowerDbm;
} rfid_ctx_t;

/**
 * @brief TMR 에러 코드를 문자열로 변환한다.
 * 입력된 TMR_ErrorCode 값을 해당 에러 이름 문자열로 반환한다.
 *
 * @param[in] code TMR 에러 코드
 * @return 에러 코드 문자열. 매칭되지 않으면 "UNKNOWN_TMR_ERROR_CODE" 반환
 */
static const char* TMR_ErrorCodeToString(TMR_ErrorCode code) {
    switch (code) {
        /* =========================
         * SUCCESS
         * ========================= */
        case ECODE_TMR_SUCCESS:
            return "ECODE_TMR_SUCCESS";

        /* =========================
         * BASIC / COMMUNICATION
         * ========================= */
        case ECODE_TMR_ERROR_TIMEOUT:
            return "ECODE_TMR_ERROR_TIMEOUT";
        case ECODE_TMR_ERROR_NO_HOST:
            return "ECODE_TMR_ERROR_NO_HOST";
        case ECODE_TMR_ERROR_LLRP:
            return "ECODE_TMR_ERROR_LLRP";
        case ECODE_TMR_ERROR_PARSE:
            return "ECODE_TMR_ERROR_PARSE";
        case ECODE_TMR_ERROR_DEVICE_RESET:
            return "ECODE_TMR_ERROR_DEVICE_RESET";
        case ECODE_TMR_ERROR_CRC_ERROR:
            return "ECODE_TMR_ERROR_CRC_ERROR";
        case ECODE_TMR_ERROR_BOOT_RESPONSE:
            return "ECODE_TMR_ERROR_BOOT_RESPONSE";

        /* =========================
         * COMMUNICATION ERROR WITH ERRNO
         * ========================= */
        case ECODE_TMR_ERROR_COMM_ERR_EPERM:
            return "ECODE_TMR_ERROR_COMM_ERR_EPERM";
        case ECODE_TMR_ERROR_COMM_ERR_ENOENT:
            return "ECODE_TMR_ERROR_COMM_ERR_ENOENT";
        case ECODE_TMR_ERROR_COMM_ERR_EIO:
            return "ECODE_TMR_ERROR_COMM_ERR_EIO";
        case ECODE_TMR_ERROR_COMM_ERR_PERMISSION_DENIED:
            return "ECODE_TMR_ERROR_COMM_ERR_PERMISSION_DENIED";
        case ECODE_TMR_ERROR_COMM_ERR_EBUSY:
            return "ECODE_TMR_ERROR_COMM_ERR_EBUSY";

        /* =========================
         * MESSAGE / COMMAND ERRORS
         * ========================= */
        case ECODE_TMR_ERROR_MSG_WRONG_NUMBER_OF_DATA:
            return "ECODE_TMR_ERROR_MSG_WRONG_NUMBER_OF_DATA";
        case ECODE_TMR_ERROR_INVALID_OPCODE:
            return "ECODE_TMR_ERROR_INVALID_OPCODE";
        case ECODE_TMR_ERROR_UNIMPLEMENTED_OPCODE:
            return "ECODE_TMR_ERROR_UNIMPLEMENTED_OPCODE";
        case ECODE_TMR_ERROR_MSG_POWER_TOO_HIGH:
            return "ECODE_TMR_ERROR_MSG_POWER_TOO_HIGH";
        case ECODE_TMR_ERROR_MSG_INVALID_FREQ_RECEIVED:
            return "ECODE_TMR_ERROR_MSG_INVALID_FREQ_RECEIVED";
        case ECODE_TMR_ERROR_MSG_INVALID_PARAMETER_VALUE:
            return "ECODE_TMR_ERROR_MSG_INVALID_PARAMETER_VALUE";
        case ECODE_TMR_ERROR_MSG_POWER_TOO_LOW:
            return "ECODE_TMR_ERROR_MSG_POWER_TOO_LOW";
        case ECODE_TMR_ERROR_UNIMPLEMENTED_FEATURE:
            return "ECODE_TMR_ERROR_UNIMPLEMENTED_FEATURE";
        case ECODE_TMR_ERROR_INVALID_BAUD_RATE:
            return "ECODE_TMR_ERROR_INVALID_BAUD_RATE";
        case ECODE_TMR_ERROR_INVALID_REGION:
            return "ECODE_TMR_ERROR_INVALID_REGION";
        case ECODE_TMR_ERROR_INVALID_LICENSE_KEY:
            return "ECODE_TMR_ERROR_INVALID_LICENSE_KEY";

        /* =========================
         * BOOTLOADER
         * ========================= */
        case ECODE_TMR_ERROR_BL_INVALID_IMAGE_CRC:
            return "ECODE_TMR_ERROR_BL_INVALID_IMAGE_CRC";
        case ECODE_TMR_ERROR_BL_INVALID_APP_END_ADDR:
            return "ECODE_TMR_ERROR_BL_INVALID_APP_END_ADDR";

        /* =========================
         * FLASH
         * ========================= */
        case ECODE_TMR_ERROR_FLASH_BAD_ERASE_PASSWORD:
            return "ECODE_TMR_ERROR_FLASH_BAD_ERASE_PASSWORD";
        case ECODE_TMR_ERROR_FLASH_BAD_WRITE_PASSWORD:
            return "ECODE_TMR_ERROR_FLASH_BAD_WRITE_PASSWORD";
        case ECODE_TMR_ERROR_FLASH_UNDEFINED_SECTOR:
            return "ECODE_TMR_ERROR_FLASH_UNDEFINED_SECTOR";
        case ECODE_TMR_ERROR_FLASH_ILLEGAL_SECTOR:
            return "ECODE_TMR_ERROR_FLASH_ILLEGAL_SECTOR";
        case ECODE_TMR_ERROR_FLASH_WRITE_TO_NON_ERASED_AREA:
            return "ECODE_TMR_ERROR_FLASH_WRITE_TO_NON_ERASED_AREA";
        case ECODE_TMR_ERROR_FLASH_WRITE_TO_ILLEGAL_SECTOR:
            return "ECODE_TMR_ERROR_FLASH_WRITE_TO_ILLEGAL_SECTOR";
        case ECODE_TMR_ERROR_FLASH_VERIFY_FAILED:
            return "ECODE_TMR_ERROR_FLASH_VERIFY_FAILED";

        /* =========================
         * TAG / PROTOCOL
         * ========================= */
        case ECODE_TMR_ERROR_NO_TAGS_FOUND:
            return "ECODE_TMR_ERROR_NO_TAGS_FOUND";
        case ECODE_TMR_ERROR_NO_PROTOCOL_DEFINED:
            return "ECODE_TMR_ERROR_NO_PROTOCOL_DEFINED";
        case ECODE_TMR_ERROR_INVALID_PROTOCOL_SPECIFIED:
            return "ECODE_TMR_ERROR_INVALID_PROTOCOL_SPECIFIED";
        case ECODE_TMR_ERROR_WRITE_PASSED_LOCK_FAILED:
            return "ECODE_TMR_ERROR_WRITE_PASSED_LOCK_FAILED";
        case ECODE_TMR_ERROR_PROTOCOL_NO_DATA_READ:
            return "ECODE_TMR_ERROR_PROTOCOL_NO_DATA_READ";
        case ECODE_TMR_ERROR_AFE_NOT_ON:
            return "ECODE_TMR_ERROR_AFE_NOT_ON";
        case ECODE_TMR_ERROR_PROTOCOL_WRITE_FAILED:
            return "ECODE_TMR_ERROR_PROTOCOL_WRITE_FAILED";
        case ECODE_TMR_ERROR_NOT_IMPLEMENTED_FOR_THIS_PROTOCOL:
            return "ECODE_TMR_ERROR_NOT_IMPLEMENTED_FOR_THIS_PROTOCOL";
        case ECODE_TMR_ERROR_PROTOCOL_INVALID_WRITE_DATA:
            return "ECODE_TMR_ERROR_PROTOCOL_INVALID_WRITE_DATA";
        case ECODE_TMR_ERROR_PROTOCOL_INVALID_ADDRESS:
            return "ECODE_TMR_ERROR_PROTOCOL_INVALID_ADDRESS";
        case ECODE_TMR_ERROR_GENERAL_TAG_ERROR:
            return "ECODE_TMR_ERROR_GENERAL_TAG_ERROR";
        case ECODE_TMR_ERROR_DATA_TOO_LARGE:
            return "ECODE_TMR_ERROR_DATA_TOO_LARGE";
        case ECODE_TMR_ERROR_PROTOCOL_INVALID_KILL_PASSWORD:
            return "ECODE_TMR_ERROR_PROTOCOL_INVALID_KILL_PASSWORD";
        case ECODE_TMR_ERROR_PROTOCOL_KILL_FAILED:
            return "ECODE_TMR_ERROR_PROTOCOL_KILL_FAILED";
        case ECODE_TMR_ERROR_PROTOCOL_BIT_DECODING_FAILED:
            return "ECODE_TMR_ERROR_PROTOCOL_BIT_DECODING_FAILED";
        case ECODE_TMR_ERROR_PROTOCOL_INVALID_EPC:
            return "ECODE_TMR_ERROR_PROTOCOL_INVALID_EPC";
        case ECODE_TMR_ERROR_PROTOCOL_INVALID_NUM_DATA:
            return "ECODE_TMR_ERROR_PROTOCOL_INVALID_NUM_DATA";

        /* =========================
         * GEN2 PROTOCOL
         * ========================= */
        case ECODE_TMR_ERROR_GEN2_PROTOCOL_OTHER_ERROR:
            return "ECODE_TMR_ERROR_GEN2_PROTOCOL_OTHER_ERROR";
        case ECODE_TMR_ERROR_GEN2_PROTOCOL_MEMORY_OVERRUN_BAD_PC:
            return "ECODE_TMR_ERROR_GEN2_PROTOCOL_MEMORY_OVERRUN_BAD_PC";
        case ECODE_TMR_ERROR_GEN2_PROTOCOL_MEMORY_LOCKED:
            return "ECODE_TMR_ERROR_GEN2_PROTOCOL_MEMORY_LOCKED";
        case ECODE_TMR_ERROR_GEN2_PROTOCOL_V2_AUTHEN_FAILED:
            return "ECODE_TMR_ERROR_GEN2_PROTOCOL_V2_AUTHEN_FAILED";
        case ECODE_TMR_ERROR_GEN2_PROTOCOL_V2_UNTRACE_FAILED:
            return "ECODE_TMR_ERROR_GEN2_PROTOCOL_V2_UNTRACE_FAILED";
        case ECODE_TMR_ERROR_GEN2_PROTOCOL_INSUFFICIENT_POWER:
            return "ECODE_TMR_ERROR_GEN2_PROTOCOL_INSUFFICIENT_POWER";
        case ECODE_TMR_ERROR_GEN2_PROTOCOL_NON_SPECIFIC_ERROR:
            return "ECODE_TMR_ERROR_GEN2_PROTOCOL_NON_SPECIFIC_ERROR";
        case ECODE_TMR_ERROR_GEN2_PROTOCOL_UNKNOWN_ERROR:
            return "ECODE_TMR_ERROR_GEN2_PROTOCOL_UNKNOWN_ERROR";

        /* =========================
         * RF / AHAL
         * ========================= */
        case ECODE_TMR_ERROR_AHAL_INVALID_FREQ:
            return "ECODE_TMR_ERROR_AHAL_INVALID_FREQ";
        case ECODE_TMR_ERROR_AHAL_CHANNEL_OCCUPIED:
            return "ECODE_TMR_ERROR_AHAL_CHANNEL_OCCUPIED";
        case ECODE_TMR_ERROR_AHAL_TRANSMITTER_ON:
            return "ECODE_TMR_ERROR_AHAL_TRANSMITTER_ON";
        case ECODE_TMR_ERROR_ANTENNA_NOT_CONNECTED:
            return "ECODE_TMR_ERROR_ANTENNA_NOT_CONNECTED";
        case ECODE_TMR_ERROR_TEMPERATURE_EXCEED_LIMITS:
            return "ECODE_TMR_ERROR_TEMPERATURE_EXCEED_LIMITS";
        case ECODE_TMR_ERROR_HIGH_RETURN_LOSS:
            return "ECODE_TMR_ERROR_HIGH_RETURN_LOSS";
        case ECODE_TMR_ERROR_INVALID_ANTENNA_CONFIG:
            return "ECODE_TMR_ERROR_INVALID_ANTENNA_CONFIG";

        /* =========================
         * TAG BUFFER
         * ========================= */
        case ECODE_TMR_ERROR_TAG_ID_BUFFER_NOT_ENOUGH_TAGS_AVAILABLE:
            return "ECODE_TMR_ERROR_TAG_ID_BUFFER_NOT_ENOUGH_TAGS_AVAILABLE";
        case ECODE_TMR_ERROR_TAG_ID_BUFFER_FULL:
            return "ECODE_TMR_ERROR_TAG_ID_BUFFER_FULL";
        case ECODE_TMR_ERROR_TAG_ID_BUFFER_REPEATED_TAG_ID:
            return "ECODE_TMR_ERROR_TAG_ID_BUFFER_REPEATED_TAG_ID";
        case ECODE_TMR_ERROR_TAG_ID_BUFFER_NUM_TAG_TOO_LARGE:
            return "ECODE_TMR_ERROR_TAG_ID_BUFFER_NUM_TAG_TOO_LARGE";
        case ECODE_TMR_ERROR_TAG_ID_BUFFER_AUTH_REQUEST:
            return "ECODE_TMR_ERROR_TAG_ID_BUFFER_AUTH_REQUEST";

        /* =========================
         * SYSTEM
         * ========================= */
        case ECODE_TMR_ERROR_SYSTEM_UNKNOWN_ERROR:
            return "ECODE_TMR_ERROR_SYSTEM_UNKNOWN_ERROR";
        case ECODE_TMR_ERROR_TM_ASSERT_FAILED:
            return "ECODE_TMR_ERROR_TM_ASSERT_FAILED";

        /* =========================
         * GENERIC API
         * ========================= */
        case ECODE_TMR_ERROR_INVALID:
            return "ECODE_TMR_ERROR_INVALID";
        case ECODE_TMR_ERROR_UNIMPLEMENTED:
            return "ECODE_TMR_ERROR_UNIMPLEMENTED";
        case ECODE_TMR_ERROR_UNSUPPORTED:
            return "ECODE_TMR_ERROR_UNSUPPORTED";
        case ECODE_TMR_ERROR_NO_ANTENNA:
            return "ECODE_TMR_ERROR_NO_ANTENNA";
        case ECODE_TMR_ERROR_READONLY:
            return "ECODE_TMR_ERROR_READONLY";
        case ECODE_TMR_ERROR_TOO_BIG:
            return "ECODE_TMR_ERROR_TOO_BIG";
        case ECODE_TMR_ERROR_NO_THREADS:
            return "ECODE_TMR_ERROR_NO_THREADS";
        case ECODE_TMR_ERROR_NO_TAGS:
            return "ECODE_TMR_ERROR_NO_TAGS";
        case ECODE_TMR_ERROR_BUFFER_OVERFLOW:
            return "ECODE_TMR_ERROR_BUFFER_OVERFLOW";
        case ECODE_TMR_ERROR_TRYAGAIN:
            return "ECODE_TMR_ERROR_TRYAGAIN";
        case ECODE_TMR_ERROR_OUT_OF_MEMORY:
            return "ECODE_TMR_ERROR_OUT_OF_MEMORY";
        case ECODE_TMR_ERROR_READER_TYPE:
            return "ECODE_TMR_ERROR_READER_TYPE";
        case ECODE_TMR_ERROR_INVALID_TAG_TYPE:
            return "ECODE_TMR_ERROR_INVALID_TAG_TYPE";
        case ECODE_TMR_ERROR_MULTIPLE_STATUS:
            return "ECODE_TMR_ERROR_MULTIPLE_STATUS";
        case ECODE_TMR_ERROR_UNEXPECTED_TAG_ID:
            return "ECODE_TMR_ERROR_UNEXPECTED_TAG_ID";
        case ECODE_TMR_ERROR_REGULATORY:
            return "ECODE_TMR_ERROR_REGULATORY";
        case ECODE_TMR_ERROR_SYSTEM_RESOURCE:
            return "ECODE_TMR_ERROR_SYSTEM_RESOURCE";

        /* =========================
         * LLRP
         * ========================= */
        case ECODE_TMR_ERROR_LLRP_READER_CONNECTION_ALREADY_OPEN:
            return "ECODE_TMR_ERROR_LLRP_READER_CONNECTION_ALREADY_OPEN";
        case ECODE_TMR_ERROR_LLRP_READER_CONNECTION_LOST_INTERNAL:
            return "ECODE_TMR_ERROR_LLRP_READER_CONNECTION_LOST_INTERNAL";
        case ECODE_TMR_ERROR_LLRP_SENDIO_ERROR:
            return "ECODE_TMR_ERROR_LLRP_SENDIO_ERROR";
        case ECODE_TMR_ERROR_LLRP_RECEIVEIO_ERROR:
            return "ECODE_TMR_ERROR_LLRP_RECEIVEIO_ERROR";
        case ECODE_TMR_ERROR_LLRP_RECEIVE_TIMEOUT:
            return "ECODE_TMR_ERROR_LLRP_RECEIVE_TIMEOUT";
        case ECODE_TMR_ERROR_LLRP_MSG_PARSE_ERROR:
            return "ECODE_TMR_ERROR_LLRP_MSG_PARSE_ERROR";
        case ECODE_TMR_ERROR_LLRP_ALREADY_CONNECTED:
            return "ECODE_TMR_ERROR_LLRP_ALREADY_CONNECTED";
        case ECODE_TMR_ERROR_LLRP_INVALID_RFMODE:
            return "ECODE_TMR_ERROR_LLRP_INVALID_RFMODE";
        case ECODE_TMR_ERROR_LLRP_UNDEFINED_VALUE:
            return "ECODE_TMR_ERROR_LLRP_UNDEFINED_VALUE";
        case ECODE_TMR_ERROR_LLRP_READER_ERROR:
            return "ECODE_TMR_ERROR_LLRP_READER_ERROR";
        case ECODE_TMR_ERROR_LLRP_READER_CONNECTION_LOST:
            return "ECODE_TMR_ERROR_LLRP_READER_CONNECTION_LOST";
        case ECODE_TMR_ERROR_LLRP_CLIENT_CONNECTION_EXISTS:
            return "ECODE_TMR_ERROR_LLRP_CLIENT_CONNECTION_EXISTS";

        default:
            return "UNKNOWN_TMR_ERROR_CODE";
    }
}

/**
 * @brief 상태 코드와 에러 문자열 OUT 매개변수를 설정한다.
 *
 * @param[out] out_status  TMR 상태 코드 출력 포인터(NULL 허용)
 * @param[out] out_errstr  상태 문자열 출력 포인터(NULL 허용)
 * @param[in]  st          TMR 상태 코드 값
 *
 * @note 문자열은 TMR_ErrorCodeToString()이 반환하는 정적 문자열 주소이다.
 */
static inline void SetOutStatusAndErr_(OUT_ uint32_t *out_status, OUT_ const char **out_errstr, IN_ TMR_Status st) {
    if (NULL != out_status)
        *out_status = (uint32_t) st;

    if (NULL != out_errstr)
        *out_errstr = TMR_ErrorCodeToString((TMR_ErrorCode) st);
}

/**
 * @brief 문자열이 NULL이거나 빈 문자열인지 확인한다.
 * @param s 검사할 문자열
 * @return NULL이거나 빈 문자열이면 1, 아니면 0
 */
static int IsNullOrEmpty_(IN_ const char *s) {
    return (NULL == s) || ('\0' == s[0]);
}

/**
 * @brief 상·하위 32비트 값을 결합하여 64비트 타임스탬프(ms)를 생성한다.
 * @param low  타임스탬프 하위 32비트 값
 * @param high 타임스탬프 상위 32비트 값
 * @return high를 상위 32비트, low를 하위 32비트로 결합한 64비트 타임스탬프(ms)
 */
static uint64_t CombineTimestampMs_(IN_ const uint32_t low, IN_ const uint32_t high) {
    return (((uint64_t) high) << 32) | (uint64_t) low;
}

/**
 * @brief rfid_tag_t 구조체 비교 함수 (qsort 용)
 * @param a 첫 번째 rfid_tag_t 포인터
 * @param b 두 번째 rfid_tag_t 포인터
 * @return RSSI 내림차순, 동일하면 ReadCount 내림차순으로 정렬
 *      b가 크면 양수, a가 크면 음수, 같으면 0
 */
static int CompareTag_(IN_ const void *a, IN_ const void *b) {
    const rfid_tag_t *x = (const rfid_tag_t *) a;
    const rfid_tag_t *y = (const rfid_tag_t *) b;

    // RSSI desc
    if (x->rssi != y->rssi)
        return (y->rssi - x->rssi);

    // ReadCount desc
    if (x->readcnt < y->readcnt)
        return 1;

    if (x->readcnt > y->readcnt)
        return -1;

    return 0;
}

/**
 * @brief RFID_REGION 값을 MercuryAPI의 TMR_Region 값으로 매핑한다.
 * @param region 변환할 RFID 지역 코드
 * @return 대응하는 TMR_Region 값 (없으면 TMR_REGION_NONE)
 */
static TMR_Region MapRegion_(IN_ const RFID_REGION region) {
    switch (region) {
        case RFID_REGION_KR2:
            return TMR_REGION_KR2;

        case RFID_REGION_US:
            // MercuryAPI의 "North America"가 US에 대응 (FCC 계열)
            return TMR_REGION_NA;

        case RFID_REGION_EU:
            return TMR_REGION_EU;

        case RFID_REGION_AUTO:
        default:
            return TMR_REGION_NONE;
    }
}

/**
 * @brief Reader가 지원하는 지역 목록에서 자동으로 Region을 선택한다.
 *
 * @param[in]  reader      초기화된 TMR_Reader 포인터
 * @param[out] out_region  선택된 Region 출력 포인터
 * @param[out] out_status  TMR 상태 코드 출력 포인터(NULL 허용)
 * @param[out] out_errstr  상태 문자열 출력 포인터(NULL 허용)
 *
 * @return RFID_RESULT_OK: 성공,
 *         RFID_RESULT_INVALID_ARG: 잘못된 인자,
 *         RFID_RESULT_REGION_FAIL: 지역 선택 실패
 *
 * @note out_errstr는 TMR_ErrorCodeToString()이 반환하는 정적 문자열 주소이다.
 */
static RFID_RESULT SelectAutoRegion_(IN_ TMR_Reader *reader
                                     , OUT_ TMR_Region *out_region
                                     , OUT_ uint32_t *out_status
                                     , OUT_ const char **out_errstr) {
    SetOutStatusAndErr_(out_status, out_errstr, TMR_SUCCESS);

    if ((NULL == reader) || (NULL == out_region))
        return RFID_RESULT_INVALID_ARG;

    TMR_Region region_buf[RFID_REGIONLIST_MAX];
    TMR_RegionList region_list;

    memset(&region_list, 0, sizeof(region_list));

    region_list.list = region_buf;
    region_list.max = (uint8_t) RFID_REGIONLIST_MAX;
    region_list.len = 0;

    const TMR_Status st = TMR_paramGet(reader, TMR_PARAM_REGION_SUPPORTEDREGIONS, &region_list);

    SetOutStatusAndErr_(out_status, out_errstr, st);

    if (TMR_SUCCESS != st)
        return RFID_RESULT_REGION_FAIL;

    const uint32_t usable_len =
            (region_list.len > region_list.max) ? region_list.max : region_list.len;

    if (0U == usable_len)
        return RFID_RESULT_REGION_FAIL;

    for (uint32_t i = 0; i < usable_len; ++i) {
        if (TMR_REGION_KR2 == region_list.list[i]) {
            *out_region = TMR_REGION_KR2;
            return RFID_RESULT_OK;
        }
    }

    *out_region = region_list.list[0];
    return RFID_RESULT_OK;
}

/**
 * @brief RFID_RESULT 값을 문자열로 변환한다.
 * @param result 변환할 RFID_RESULT 코드
 * @return 해당 결과 코드에 대응되는 문자열, 정의되지 않은 값은 "RFID_RESULT_INTERNAL_ERROR"
 */
const char* rfid_result_to_string(IN_ const RFID_RESULT result) {
    switch (result) {
        case RFID_RESULT_OK:
            return "RFID_RESULT_OK";

        case RFID_RESULT_DISABLED:
            return "RFID_RESULT_DISABLED";

        case RFID_RESULT_INVALID_ARG:
            return "RFID_RESULT_INVALID_ARG";

        case RFID_RESULT_NOT_INITIALIZED:
            return "RFID_RESULT_NOT_INITIALIZED";

        case RFID_RESULT_CONNECT_FAIL:
            return "RFID_RESULT_CONNECT_FAIL";

        case RFID_RESULT_REGION_FAIL:
            return "RFID_RESULT_REGION_FAIL";

        case RFID_RESULT_PLAN_FAIL:
            return "RFID_RESULT_PLAN_FAIL";

        case RFID_RESULT_READ_FAIL:
            return "RFID_RESULT_READ_FAIL";

        case RFID_RESULT_INTERNAL_ERROR:
        default:
            return "RFID_RESULT_INTERNAL_ERROR";
    }
}

/**
 * @brief RFID_REGION 값을 문자열로 변환한다.
 * @param region 변환할 RFID_REGION 코드
 * @return 해당 지역 코드에 대응되는 문자열, 정의되지 않은 값은 "RFID_REGION_UNKNOWN"
 */
const char* rfid_region_to_string(IN_ const RFID_REGION region) {
    switch (region) {
        case RFID_REGION_AUTO:
            return "RFID_REGION_AUTO";

        case RFID_REGION_KR2:
            return "RFID_REGION_KR2";

        case RFID_REGION_US:
            return "RFID_REGION_US";

        case RFID_REGION_EU:
            return "RFID_REGION_EU";

        default:
            return "RFID_REGION_UNKNOWN";
    }
}

/**
 * @brief rfid_init 파라미터 유효성을 검사한다.
 * @param out_ctx 출력 컨텍스트 포인터(주소)
 * @param params 초기화 파라미터
 * @return 유효하면 RFID_RESULT_OK, 아니면 해당 오류 코드
 */
static RFID_RESULT ValidateInitParams_(OUT_ rfid_ctx_t **out_ctx, IN_ const rfid_init_params_t *params) {
    if ((NULL == out_ctx) || (NULL == params))
        return RFID_RESULT_INVALID_ARG;
    if (0 == params->rfid_enable)
        return RFID_RESULT_DISABLED;
    if (0 != IsNullOrEmpty_(params->uri))
        return RFID_RESULT_INVALID_ARG;
    if ((NULL == params->antennas) || (params->antenna_count <= 0) || (params->antenna_count > (int) RFID_MAX_ANTENNAS))
        return RFID_RESULT_INVALID_ARG;
    return RFID_RESULT_OK;
}

/**
 * @brief Reader 자원을 해제한다(생성 성공 후에만 의미 있음).
 *
 * @param[in]  ctx         RFID 컨텍스트
 * @param[out] out_status  TMR 상태코드(선택). NULL이면 무시.
 * @param[out] out_errstr  상태 문자열(선택). NULL이면 무시.
 *
 * @return RFID_RESULT_OK 성공(또는 해제할 대상 없음), 실패 시 RFID_RESULT_INTERNAL_ERROR
 */
static RFID_RESULT DestroyReader_(IN_ rfid_ctx_t *ctx, OUT_ uint32_t *out_status, OUT_ const char **out_errstr) {
    SetOutStatusAndErr_(out_status, out_errstr, TMR_SUCCESS);

    if ((NULL == ctx) || (0 == ctx->reader_created))
        return RFID_RESULT_OK;

    TMR_Status st = TMR_destroy(&ctx->reader);

    SetOutStatusAndErr_(out_status, out_errstr, st);

    ctx->reader_created = 0;

    return (st == TMR_SUCCESS) ? RFID_RESULT_OK : RFID_RESULT_INTERNAL_ERROR;
}

/**
 * @brief Reader를 생성한다.
 *
 * @param[in]  ctx         RFID 컨텍스트
 * @param[in]  uri         Reader URI(빈 문자열 불가)
 * @param[out] out_status  TMR 상태 코드 출력 포인터(NULL 허용)
 * @param[out] out_errstr  상태 문자열 출력 포인터(NULL 허용)
 *
 * @return RFID_RESULT_OK: 성공,
 *         RFID_RESULT_INVALID_ARG: 잘못된 인자,
 *         RFID_RESULT_CONNECT_FAIL: 생성 실패
 *
 * @note out_errstr는 TMR_ErrorCodeToString()이 반환하는 정적 문자열 주소이다.
 */
static RFID_RESULT CreateReader_(IN_ rfid_ctx_t *ctx
                                 , IN_ const char *uri
                                 , OUT_ uint32_t *out_status
                                 , OUT_ const char **out_errstr) {
    SetOutStatusAndErr_(out_status, out_errstr, TMR_SUCCESS);

    if ((NULL == ctx) || 0 != IsNullOrEmpty_(uri))
        return RFID_RESULT_INVALID_ARG;

    const TMR_Status st = TMR_create(&ctx->reader, uri);

    SetOutStatusAndErr_(out_status, out_errstr, st);

    if (TMR_SUCCESS != st) {
        ctx->reader_created = 0;
        return RFID_RESULT_CONNECT_FAIL;
    }

    ctx->reader_created = 1;
    return RFID_RESULT_OK;
}

/**
 * @brief 생성된 Reader에 연결한다.
 *
 * @param[in]  ctx         RFID 컨텍스트
 * @param[out] out_status  TMR 상태 코드 출력 포인터(NULL 허용)
 * @param[out] out_errstr  상태 문자열 출력 포인터(NULL 허용)
 *
 * @return RFID_RESULT_OK: 성공,
 *         RFID_RESULT_INVALID_ARG: 잘못된 인자,
 *         RFID_RESULT_CONNECT_FAIL: 연결 실패
 *
 * @note out_errstr는 TMR_ErrorCodeToString()이 반환하는 정적 문자열 주소이다.
 */
static RFID_RESULT ConnectReader_(IN_ rfid_ctx_t *ctx, OUT_ uint32_t *out_status, OUT_ const char **out_errstr) {
    SetOutStatusAndErr_(out_status, out_errstr, TMR_SUCCESS);

    if ((NULL == ctx) || (0 == ctx->reader_created))
        return RFID_RESULT_INVALID_ARG;

    const TMR_Status st = TMR_connect(&ctx->reader);

    SetOutStatusAndErr_(out_status, out_errstr, st);

    return (st == TMR_SUCCESS) ? RFID_RESULT_OK : RFID_RESULT_CONNECT_FAIL;
}

/**
 * @brief Region을 결정하고 Reader에 설정한다.
 *
 * @param[in]  reader      MercuryAPI Reader 핸들
 * @param[in]  region      사용자 지정 Region
 * @param[out] out_status  TMR 상태 코드 출력 포인터(NULL 허용)
 * @param[out] out_errstr  상태 문자열 출력 포인터(NULL 허용)
 *
 * @return RFID_RESULT_OK: 성공,
 *         RFID_RESULT_INVALID_ARG: 잘못된 인자,
 *         RFID_RESULT_REGION_FAIL: Region 매핑/설정 실패,
 *         기타: SelectAutoRegion_() 결과 반환
 *
 * @note out_errstr는 TMR_ErrorCodeToString()이 반환하는 정적 문자열 주소이다.
 */
static RFID_RESULT ConfigureRegion_(IN_ TMR_Reader *reader
                                    , IN_ const RFID_REGION region
                                    , OUT_ uint32_t *out_status
                                    , OUT_ const char **out_errstr) {
    SetOutStatusAndErr_(out_status, out_errstr, TMR_SUCCESS);

    if (NULL == reader)
        return RFID_RESULT_INVALID_ARG;

    TMR_Region region_to_set = TMR_REGION_NONE;

    if (RFID_REGION_AUTO == region) {
        const RFID_RESULT sel = SelectAutoRegion_(reader, &region_to_set, out_status, out_errstr);
        if (RFID_RESULT_OK != sel)
            return sel;
    }
    else {
        region_to_set = MapRegion_(region);
        if (TMR_REGION_NONE == region_to_set)
            return RFID_RESULT_REGION_FAIL;
    }

    const TMR_Status st = TMR_paramSet(reader, TMR_PARAM_REGION_ID, &region_to_set);

    SetOutStatusAndErr_(out_status, out_errstr, st);

    return (st == TMR_SUCCESS) ? RFID_RESULT_OK : RFID_RESULT_REGION_FAIL;
}

/**
 * @brief GEN2 Read Plan(안테나 리스트 포함)을 설정한다.
 *
 * @param[in]  reader      MercuryAPI Reader 핸들
 * @param[in]  antennas    안테나 번호 배열(1..N)
 * @param[in]  antenna_count 안테나 개수
 * @param[in]  plan_timeout_ms plan timeout(ms), 0 이하이면 기본값 사용
 * @param[out] out_status  TMR 상태 코드 출력 포인터(NULL 허용)
 * @param[out] out_errstr  상태 문자열 출력 포인터(NULL 허용)
 *
 * @return RFID_RESULT_OK: 성공,
 *         RFID_RESULT_INVALID_ARG: 인자 오류,
 *         RFID_RESULT_PLAN_FAIL: 설정 실패
 *
 * @note out_errstr는 TMR_ErrorCodeToString()이 반환하는 정적 문자열 주소이다.
 */
static RFID_RESULT ConfigureReadPlan_(IN_ TMR_Reader *reader
                                      , IN_ const int *antennas
                                      , IN_ const int antenna_count
                                      , IN_ const int plan_timeout_ms
                                      , OUT_ uint32_t *out_status
                                      , OUT_ const char **out_errstr) {
    SetOutStatusAndErr_(out_status, out_errstr, TMR_SUCCESS);

    if ((NULL == reader) || (NULL == antennas) || (antenna_count <= 0) || (antenna_count > (int) RFID_MAX_ANTENNAS))
        return RFID_RESULT_INVALID_ARG;

    uint8_t antenna_list_u8[RFID_MAX_ANTENNAS];
    for (int i = 0; i < antenna_count; ++i) {
        const int ant = antennas[i];
        if (ant <= 0 || ant > 255)
            return RFID_RESULT_INVALID_ARG;

        antenna_list_u8[i] = (uint8_t) ant;
    }

    const uint32_t readTime = (plan_timeout_ms > 0)
                                  ? (uint32_t) plan_timeout_ms
                                  : (uint32_t) RFID_DEFAULT_PLAN_READTIME;

    TMR_ReadPlan plan;
    memset(&plan, 0, sizeof(plan));

    const TMR_Status st1 = TMR_RP_init_simple(&plan
                                              , (uint8_t) antenna_count
                                              , antenna_list_u8
                                              , TMR_TAG_PROTOCOL_GEN2
                                              , readTime);
    if (TMR_SUCCESS != st1) {
        SetOutStatusAndErr_(out_status, out_errstr, st1);
        return RFID_RESULT_PLAN_FAIL;
    }

    const TMR_Status st2 = TMR_paramSet(reader, TMR_PARAM_READ_PLAN, &plan);
    SetOutStatusAndErr_(out_status, out_errstr, st2);

    return (st2 == TMR_SUCCESS) ? RFID_RESULT_OK : RFID_RESULT_PLAN_FAIL;
}

/**
 * @brief 읽기 전력(dBm)이 양수인 경우 Reader에 설정한다.
 *
 * @param[in]  reader MercuryAPI Reader 핸들
 * @param[in]  read_power_cdbm 읽기 전력(cdBm), 0 이하이면 설정하지 않음
 * @param[out] out_status TMR 상태 코드 출력 포인터(NULL 허용)
 * @param[out] out_errstr 상태 문자열 출력 포인터(NULL 허용)
 *
 * @return RFID_RESULT_OK: 성공,
 *         RFID_RESULT_INTERNAL_ERROR: 설정 실패
 *
 * @note out_errstr는 TMR_ErrorCodeToString()이 반환하는 정적 문자열 주소이다.
 */
static RFID_RESULT ConfigureWritePower_(IN_ TMR_Reader *reader
                                       , IN_ const int read_power_cdbm
                                       , OUT_ uint32_t *out_status
                                       , OUT_ const char **out_errstr) {
    SetOutStatusAndErr_(out_status, out_errstr, TMR_SUCCESS);
    if (read_power_cdbm <= 0)
        return RFID_RESULT_OK;

    const int32_t power_cdbm = (int32_t) read_power_cdbm;
    const TMR_Status st = TMR_paramSet(reader, TMR_PARAM_RADIO_READPOWER, &power_cdbm);

    SetOutStatusAndErr_(out_status, out_errstr, st);

    return (st == TMR_SUCCESS) ? RFID_RESULT_OK : RFID_RESULT_INTERNAL_ERROR;
}

/**
 * @brief RFID Reader를 생성/연결하고 Region 및 Read Plan을 설정하여 컨텍스트를 초기화한다.
 * @param[out] out_ctx 초기화 완료 후 생성된 컨텍스트 포인터를 반환받을 출력 포인터
 * @param[in]  params 초기화 파라미터(사용 여부, URI, Region, 안테나 목록/개수, plan timeout, read power 등)
 * @param[out] out_status TMR 상태 코드 출력 포인터(NULL 허용)
 * @param[out] out_errstr 상태 문자열 출력 포인터(NULL 허용)
 * @return 성공 시 RFID_RESULT_OK, 비활성화 시 RFID_RESULT_DISABLED, 인자 오류 시 RFID_RESULT_INVALID_ARG,
 *         연결 실패 시 RFID_RESULT_CONNECT_FAIL, Region 설정 실패 시 RFID_RESULT_REGION_FAIL,
 *         Read plan 설정 실패 시 RFID_RESULT_PLAN_FAIL, 메모리/기타 내부 오류 시 RFID_RESULT_INTERNAL_ERROR
 *
 * @note out_errstr는 TMR_ErrorCodeToString()이 반환하는 정적 문자열 주소이다.
 */
RFID_RESULT rfid_init(OUT_ rfid_ctx_t **out_ctx
                      , IN_ const rfid_init_params_t *params
                      , OUT_ uint32_t *out_status
                      , OUT_ const char **out_errstr) {
    SetOutStatusAndErr_(out_status, out_errstr, TMR_SUCCESS);

    // ------------------------------
    // 파라미터 유효성 검사
    // ------------------------------
    RFID_RESULT ret = ValidateInitParams_(out_ctx, params);
    if (RFID_RESULT_OK != ret) {
        SetOutStatusAndErr_(out_status, out_errstr, -1);
        return ret;
    }

    *out_ctx = NULL;

    // ------------------------------
    // 컨텍스트 할당 및 초기화
    // ------------------------------
    rfid_ctx_t *ctx = (rfid_ctx_t *) malloc(sizeof(*ctx));
    if (NULL == ctx) {
        SetOutStatusAndErr_(out_status, out_errstr, -1);
        return RFID_RESULT_INTERNAL_ERROR;
    }

    memset(ctx, 0, sizeof(*ctx));
    ctx->initialized = 0;
    ctx->reader_created = 0;
    ctx->region = params->region;
    ctx->readPowerDbm = params->write_power_cdbm;

    // ------------------------------
    // Reader 생성/연결 및 설정
    // ------------------------------
    ret = CreateReader_(ctx, params->uri, out_status, out_errstr);
    if (RFID_RESULT_OK != ret) {
        free(ctx);
        return ret;
    }

    // Reader 연결
    ret = ConnectReader_(ctx, out_status, out_errstr);
    if (RFID_RESULT_OK != ret) {
        DestroyReader_(ctx, NULL, NULL);
        free(ctx);
        return ret;
    }

    // Region 설정
    ret = ConfigureRegion_(&ctx->reader, params->region, out_status, out_errstr);
    if (RFID_RESULT_OK != ret) {
        DestroyReader_(ctx, NULL, NULL);
        free(ctx);
        return ret;
    }

    // Read Plan 설정
    ret = ConfigureReadPlan_(&ctx->reader
                             , params->antennas
                             , params->antenna_count
                             , params->plan_timeout_ms
                             , out_status
                             , out_errstr);
    if (RFID_RESULT_OK != ret) {
        DestroyReader_(ctx, NULL, NULL);
        free(ctx);
        return ret;
    }

    // 쓰기 전력 설정
    ret = ConfigureWritePower_(&ctx->reader, params->write_power_cdbm, out_status, out_errstr);
    if (RFID_RESULT_OK != ret) {
        DestroyReader_(ctx, NULL, NULL);
        free(ctx);
        return ret;
    }

    // ------------------------------
    // 초기화 완료
    // ------------------------------
    ctx->initialized = 1;
    *out_ctx = ctx;
    return RFID_RESULT_OK;
}

/**
 * @brief RFID 컨텍스트를 해제한다.
 *
 * @param[in,out] inout_ctx 해제할 컨텍스트 포인터 주소
 * @param[out] out_status TMR 상태 코드 출력 포인터(NULL 허용)
 * @param[out] out_errstr 상태 문자열 출력 포인터(NULL 허용)
 *
 * @return RFID_RESULT_OK: 성공,
 *         RFID_RESULT_INVALID_ARG: 잘못된 인자
 *
 * @note out_errstr는 TMR_ErrorCodeToString()이 반환하는 정적 문자열 주소이다.
 */
RFID_RESULT rfid_deinit(INOUT_ rfid_ctx_t **inout_ctx, OUT_ uint32_t *out_status, OUT_ const char **out_errstr) {
    SetOutStatusAndErr_(out_status, out_errstr, TMR_SUCCESS);

    if (NULL == inout_ctx) {
        SetOutStatusAndErr_(out_status, out_errstr, -1);
        return RFID_RESULT_INVALID_ARG;
    }

    if (NULL == *inout_ctx) {
        SetOutStatusAndErr_(out_status, out_errstr, TMR_SUCCESS);
        return RFID_RESULT_OK;
    }

    rfid_ctx_t *ctx = *inout_ctx;

    if (ctx->initialized) {
        const TMR_Status st = TMR_destroy(&ctx->reader);
        SetOutStatusAndErr_(out_status, out_errstr, st);
    }

    free(ctx);
    *inout_ctx = NULL;
    return RFID_RESULT_OK;
}

/**
 * @brief Write Power(centi-dBm)를 변경한다. (wrapper)
 *
 * - ConfigureWritePower_() 내부 구현을 직접 노출하지 않고, 인자 검증/전처리를 포함한 API를 제공한다.
 * - read_power_cdbm <= 0 이면 "기본값 사용"으로 간주하며, 장치 파라미터를 변경하지 않고 성공을 반환한다.
 * - ctx가 NULL이거나 초기화되지 않은 경우, RFID_RESULT_INVALID_ARG / RFID_RESULT_NOT_INITIALIZED 를 반환한다.
 *
 * @param[in]  ctx RFID 컨텍스트(in)
 * @param[in]  read_power_cdbm 읽기 전력(centi-dBm). 예: 3000 == 30.00 dBm
 * @param[out] out_status TMR 상태 코드(out). NULL 허용. SDK 외부 오류 시 -1.
 * @param[out] out_errstr 상태 문자열(out). NULL 허용.
 *
 * @return RFID_RESULT 결과 코드
 */
RFID_RESULT rfid_set_write_power(IN_ rfid_ctx_t *ctx
                               , IN_ const int read_power_cdbm
                               , OUT_ uint32_t *out_status
                               , OUT_ const char **out_errstr) {
    SetOutStatusAndErr_(out_status, out_errstr, TMR_SUCCESS);

    if (NULL == ctx) {
        if (NULL != out_status) *out_status = (uint32_t) TMR_ERROR_INVALID;
        if (NULL != out_errstr) *out_errstr = "RFID_INVALID_ARG(ctx)";
        return RFID_RESULT_INVALID_ARG;
    }

    if (!ctx->initialized) {
        if (NULL != out_status) *out_status = (uint32_t) TMR_ERROR_INVALID;
        if (NULL != out_errstr) *out_errstr = "RFID_NOT_INITIALIZED";
        return RFID_RESULT_NOT_INITIALIZED;
    }

    /* "기본값 사용" 요청: 장치 파라미터를 건드리지 않음 */
    if (read_power_cdbm <= 0) {
        ctx->readPowerDbm = 0;
        return RFID_RESULT_OK;
    }

    /* 방어적 범위 체크(centi-dBm). 대부분 리더는 0~3000(=30dBm) 범위를 사용한다. */
    if (read_power_cdbm > 3000) {
        if (NULL != out_status) *out_status = (uint32_t) TMR_ERROR_INVALID;
        if (NULL != out_errstr) *out_errstr = "RFID_INVALID_ARG(read_power_cdbm out of range)";
        return RFID_RESULT_INVALID_ARG;
    }

    const RFID_RESULT ret = ConfigureWritePower_(&ctx->reader, read_power_cdbm, out_status, out_errstr);
    if (RFID_RESULT_OK == ret) {
        ctx->readPowerDbm = read_power_cdbm;
    }
    return ret;
}


/**
 * @brief Reader에서 태그를 읽어 out_tags에 저장한다.
 *
 * @param[in]  ctx RFID 컨텍스트
 * @param[in]  read_timeout_ms 읽기 타임아웃(ms), 0 이상
 * @param[out] out_tags 태그 결과 버퍼
 * @param[in]  tag_capacity out_tags 용량(개수)
 * @param[out] out_count 읽힌 태그 개수
 * @param[out] out_status TMR 상태 코드 출력 포인터(NULL 허용)
 * @param[out] out_errstr 상태 문자열 출력 포인터(NULL 허용)
 *
 * @return RFID_RESULT_OK: 성공(태그 0개 포함),
 *         RFID_RESULT_INVALID_ARG: 인자 오류,
 *         RFID_RESULT_NOT_INITIALIZED: 미초기화,
 *         RFID_RESULT_READ_FAIL: 읽기 실패
 *
 * @note out_errstr는 TMR_ErrorCodeToString()이 반환하는 정적 문자열 주소이다.
 * @note SDK 외부 오류(인자 오류 등)는 out_status=-1로 설정될 수 있다.
 */
RFID_RESULT rfid_read(IN_ rfid_ctx_t *ctx
                      , IN_ const int *antennas
                      , IN_ const int antenna_count
                      , IN_ const int read_timeout_ms
                      , OUT_ rfid_tag_t *out_tags
                      , IN_ const int tag_capacity
                      , OUT_ int *out_count
                      , OUT_ uint32_t *out_status
                      , OUT_ const char **out_errstr) {
    SetOutStatusAndErr_(out_status, out_errstr, TMR_SUCCESS);

    if ((NULL == ctx) || (NULL == out_tags) || (NULL == out_count)) {
        SetOutStatusAndErr_(out_status, out_errstr, -1);
        return RFID_RESULT_INVALID_ARG;
    }

    if (1 != ctx->initialized) {
        SetOutStatusAndErr_(out_status, out_errstr, -1);
        return RFID_RESULT_NOT_INITIALIZED;
    }

    if ((tag_capacity <= 0) || (read_timeout_ms < 0)) {
        SetOutStatusAndErr_(out_status, out_errstr, -1);
        return RFID_RESULT_INVALID_ARG;
    }

    *out_count = 0;

    /* TMR_read() 호출 전 ReadPlan 재설정 */
    const RFID_RESULT st_plan = ConfigureReadPlan_(&ctx->reader
                                                   , antennas
                                                   , antenna_count
                                                   , read_timeout_ms
                                                   , out_status
                                                   , out_errstr);
    if (RFID_RESULT_OK != st_plan)
        return RFID_RESULT_READ_FAIL;

    int32_t tag_count_from_reader = 0;
    const TMR_Status st_read = TMR_read(&ctx->reader, (uint32_t) read_timeout_ms, &tag_count_from_reader);
    SetOutStatusAndErr_(out_status, out_errstr, st_read);
    if (TMR_SUCCESS != st_read)
        return RFID_RESULT_READ_FAIL;

    // hasMoreTags / getNextTag 로 결과를 가져온다.
    while (TMR_SUCCESS == TMR_hasMoreTags(&ctx->reader)) {
        if (*out_count >= tag_capacity) {
            // 버퍼 용량 초과: 이후 태그는 무시 (정책: OK 반환, count는 capacity로 제한)
            TMR_TagReadData dummy;
            (void) TMR_getNextTag(&ctx->reader, &dummy);
            continue;
        }

        TMR_TagReadData trd;
        memset(&trd, 0, sizeof(trd));

        const TMR_Status st_next = TMR_getNextTag(&ctx->reader, &trd);
        SetOutStatusAndErr_(out_status, out_errstr, st_next);
        if (TMR_SUCCESS != st_next)
            return RFID_RESULT_READ_FAIL;

        rfid_tag_t *dst = &out_tags[*out_count];
        memset(dst, 0, sizeof(*dst));

        // EPC bytes -> hex string
        const uint32_t max_bytes = (uint32_t) ((RFID_EPC_MAX_LEN - 1) / 2);
        const uint32_t use_bytes = (trd.tag.epcByteCount > max_bytes) ? max_bytes : trd.tag.epcByteCount;

        // TMR_bytesToHex는 null-terminated 문자열을 만들어 준다.
        TMR_bytesToHex(trd.tag.epc, use_bytes, dst->epc);

        dst->rssi = (int) trd.rssi;
        dst->readcnt = (uint32_t) trd.readCount;
        dst->antenna = (int) trd.antenna;
        dst->ts = CombineTimestampMs_(trd.timestampLow, trd.timestampHigh);

        (*out_count)++;
    }

    // 태그가 없으면 out_count=0 이고 OK 반환 (정책)
    if (*out_count > 1)
        qsort(out_tags, (size_t) (*out_count), sizeof(rfid_tag_t), CompareTag_);

    return RFID_RESULT_OK;
}
