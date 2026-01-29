#ifndef RFID_TYPES_H_
#define RFID_TYPES_H_

#ifdef __cplusplus
extern "C" {

#endif

#include <stdint.h>

// 입력/출력/입출력 방향 표기 매크로(문서/가독성용). 다른 환경에서 이미 정의되어 있으면 재정의하지 않습니다.
#ifndef IN_
#define IN_
#endif

#ifndef OUT_
#define OUT_
#endif

#ifndef INOUT_
#define INOUT_
#endif

// EPC 문자열 최대 길이(여유 포함). MercuryAPI는 EPC를 bytes로도 제공하므로 래퍼에서 문자열로 변환해 저장합니다.
#define RFID_EPC_MAX_LEN (128)

/**
 * @brief RFID API 공통 결과 코드
 */
typedef enum RFID_RESULT {
    RFID_RESULT_OK = 0, // 성공
    RFID_RESULT_DISABLED, // rfid_enable==0 으로 기능 비활성
    RFID_RESULT_INVALID_ARG, // NULL 포인터, 범위 오류 등 잘못된 인자
    RFID_RESULT_NOT_INITIALIZED, // init 이전 상태에서 read 등 호출
    RFID_RESULT_CONNECT_FAIL, // reader create/connect 실패
    RFID_RESULT_REGION_FAIL, // region 설정/조회 실패
    RFID_RESULT_PLAN_FAIL, // read plan 설정 실패
    RFID_RESULT_READ_FAIL, // read 수행 실패
    RFID_RESULT_INTERNAL_ERROR // 그 외 내부 오류
} RFID_RESULT;


/**
 * @brief RFID API 세부 결과 코드
 */
typedef enum TMR_ErrorCode
{
    /* =========================================================
     * SUCCESS
     * ========================================================= */
    ECODE_TMR_SUCCESS = 0,

    /* =========================================================
     * BASIC COMMUNICATION / SYSTEM RESPONSE
     * ========================================================= */
    ECODE_TMR_ERROR_TIMEOUT               = 16777217,
    ECODE_TMR_ERROR_NO_HOST               = 16777218,
    ECODE_TMR_ERROR_LLRP                  = 16777219,
    ECODE_TMR_ERROR_PARSE                 = 16777220,
    ECODE_TMR_ERROR_DEVICE_RESET          = 16777221,
    ECODE_TMR_ERROR_CRC_ERROR             = 16777222,
    ECODE_TMR_ERROR_BOOT_RESPONSE         = 16777223,

    /* =========================================================
     * COMMUNICATION ERROR WITH ERRNO (add: developer define)
     * ========================================================= */
    ECODE_TMR_ERROR_COMM_ERR_EPERM        = 16809985,   /* 0x01008001 : Operation not permitted */
    ECODE_TMR_ERROR_COMM_ERR_ENOENT       = 16809986,   /* 0x01008002 : No such file or directory */
    ECODE_TMR_ERROR_COMM_ERR_EIO          = 16809989,   /* 0x01008005 : I/O error */
    ECODE_TMR_ERROR_COMM_ERR_PERMISSION_DENIED = 16809997,   /* 0x0100800D : Permission denied */
    ECODE_TMR_ERROR_COMM_ERR_EBUSY        = 16810000,   /* 0x01008010 : Device or resource busy */

    /* =========================================================
     * MESSAGE / COMMAND VALIDATION
     * ========================================================= */
    ECODE_TMR_ERROR_MSG_WRONG_NUMBER_OF_DATA        = 33554688,
    ECODE_TMR_ERROR_INVALID_OPCODE                 = 33554689,
    ECODE_TMR_ERROR_UNIMPLEMENTED_OPCODE           = 33554690,
    ECODE_TMR_ERROR_MSG_POWER_TOO_HIGH             = 33554691,
    ECODE_TMR_ERROR_MSG_INVALID_FREQ_RECEIVED      = 33554692,
    ECODE_TMR_ERROR_MSG_INVALID_PARAMETER_VALUE    = 33554693,
    ECODE_TMR_ERROR_MSG_POWER_TOO_LOW              = 33554694,
    ECODE_TMR_ERROR_UNIMPLEMENTED_FEATURE          = 33554697,
    ECODE_TMR_ERROR_INVALID_BAUD_RATE              = 33554698,
    ECODE_TMR_ERROR_INVALID_REGION                 = 33554699,
    ECODE_TMR_ERROR_INVALID_LICENSE_KEY            = 33554700,

    /* =========================================================
     * BOOTLOADER
     * ========================================================= */
    ECODE_TMR_ERROR_BL_INVALID_IMAGE_CRC            = 33554944,
    ECODE_TMR_ERROR_BL_INVALID_APP_END_ADDR         = 33554945,

    /* =========================================================
     * FLASH MEMORY
     * ========================================================= */
    ECODE_TMR_ERROR_FLASH_BAD_ERASE_PASSWORD        = 33555200,
    ECODE_TMR_ERROR_FLASH_BAD_WRITE_PASSWORD        = 33555201,
    ECODE_TMR_ERROR_FLASH_UNDEFINED_SECTOR          = 33555202,
    ECODE_TMR_ERROR_FLASH_ILLEGAL_SECTOR            = 33555203,
    ECODE_TMR_ERROR_FLASH_WRITE_TO_NON_ERASED_AREA  = 33555204,
    ECODE_TMR_ERROR_FLASH_WRITE_TO_ILLEGAL_SECTOR   = 33555205,
    ECODE_TMR_ERROR_FLASH_VERIFY_FAILED             = 33555206,

    /* =========================================================
     * TAG / PROTOCOL GENERAL
     * ========================================================= */
    ECODE_TMR_ERROR_NO_TAGS_FOUND                   = 33555456,
    ECODE_TMR_ERROR_NO_PROTOCOL_DEFINED             = 33555457,
    ECODE_TMR_ERROR_INVALID_PROTOCOL_SPECIFIED      = 33555458,
    ECODE_TMR_ERROR_WRITE_PASSED_LOCK_FAILED        = 33555459,
    ECODE_TMR_ERROR_PROTOCOL_NO_DATA_READ           = 33555460,
    ECODE_TMR_ERROR_AFE_NOT_ON                      = 33555461,
    ECODE_TMR_ERROR_PROTOCOL_WRITE_FAILED           = 33555462,
    ECODE_TMR_ERROR_NOT_IMPLEMENTED_FOR_THIS_PROTOCOL = 33555463,
    ECODE_TMR_ERROR_PROTOCOL_INVALID_WRITE_DATA     = 33555464,
    ECODE_TMR_ERROR_PROTOCOL_INVALID_ADDRESS        = 33555465,
    ECODE_TMR_ERROR_GENERAL_TAG_ERROR               = 33555466,
    ECODE_TMR_ERROR_DATA_TOO_LARGE                  = 33555467,
    ECODE_TMR_ERROR_PROTOCOL_INVALID_KILL_PASSWORD  = 33555468,
    ECODE_TMR_ERROR_PROTOCOL_KILL_FAILED            = 33555470,
    ECODE_TMR_ERROR_PROTOCOL_BIT_DECODING_FAILED    = 33555471,
    ECODE_TMR_ERROR_PROTOCOL_INVALID_EPC            = 33555472,
    ECODE_TMR_ERROR_PROTOCOL_INVALID_NUM_DATA       = 33555473,

    /* =========================================================
     * GEN2 PROTOCOL
     * ========================================================= */
    ECODE_TMR_ERROR_GEN2_PROTOCOL_OTHER_ERROR               = 33555488,
    ECODE_TMR_ERROR_GEN2_PROTOCOL_MEMORY_OVERRUN_BAD_PC     = 33555491,
    ECODE_TMR_ERROR_GEN2_PROTOCOL_MEMORY_LOCKED             = 33555492,
    ECODE_TMR_ERROR_GEN2_PROTOCOL_V2_AUTHEN_FAILED          = 33555493,
    ECODE_TMR_ERROR_GEN2_PROTOCOL_V2_UNTRACE_FAILED         = 33555494,
    ECODE_TMR_ERROR_GEN2_PROTOCOL_INSUFFICIENT_POWER        = 33555499,
    ECODE_TMR_ERROR_GEN2_PROTOCOL_NON_SPECIFIC_ERROR        = 33555503,
    ECODE_TMR_ERROR_GEN2_PROTOCOL_UNKNOWN_ERROR             = 33555504,

    /* =========================================================
     * RF / AHAL (HARDWARE)
     * ========================================================= */
    ECODE_TMR_ERROR_AHAL_INVALID_FREQ                = 33555712,
    ECODE_TMR_ERROR_AHAL_CHANNEL_OCCUPIED            = 33555713,
    ECODE_TMR_ERROR_AHAL_TRANSMITTER_ON              = 33555714,
    ECODE_TMR_ERROR_ANTENNA_NOT_CONNECTED            = 33555715,
    ECODE_TMR_ERROR_TEMPERATURE_EXCEED_LIMITS        = 33555716,
    ECODE_TMR_ERROR_HIGH_RETURN_LOSS                 = 33555717,
    ECODE_TMR_ERROR_INVALID_ANTENNA_CONFIG           = 33555719,

    /* =========================================================
     * TAG ID BUFFER
     * ========================================================= */
    ECODE_TMR_ERROR_TAG_ID_BUFFER_NOT_ENOUGH_TAGS_AVAILABLE = 33555968,
    ECODE_TMR_ERROR_TAG_ID_BUFFER_FULL                      = 33555969,
    ECODE_TMR_ERROR_TAG_ID_BUFFER_REPEATED_TAG_ID           = 33555970,
    ECODE_TMR_ERROR_TAG_ID_BUFFER_NUM_TAG_TOO_LARGE         = 33555971,
    ECODE_TMR_ERROR_TAG_ID_BUFFER_AUTH_REQUEST              = 33555972,

    /* =========================================================
     * SYSTEM INTERNAL
     * ========================================================= */
    ECODE_TMR_ERROR_SYSTEM_UNKNOWN_ERROR            = 33586944,
    ECODE_TMR_ERROR_TM_ASSERT_FAILED                = 33586945,

    /* =========================================================
     * GENERIC API ERRORS
     * ========================================================= */
    ECODE_TMR_ERROR_INVALID                         = 50331649,
    ECODE_TMR_ERROR_UNIMPLEMENTED                   = 50331650,
    ECODE_TMR_ERROR_UNSUPPORTED                     = 50331651,
    ECODE_TMR_ERROR_NO_ANTENNA                      = 50331652,
    ECODE_TMR_ERROR_READONLY                        = 50331653,
    ECODE_TMR_ERROR_TOO_BIG                         = 50331654,
    ECODE_TMR_ERROR_NO_THREADS                      = 50331655,
    ECODE_TMR_ERROR_NO_TAGS                         = 50331656,
    ECODE_TMR_ERROR_BUFFER_OVERFLOW                 = 50331657,
    ECODE_TMR_ERROR_TRYAGAIN                        = 50331658,
    ECODE_TMR_ERROR_OUT_OF_MEMORY                   = 50331659,
    ECODE_TMR_ERROR_READER_TYPE                     = 50331660,
    ECODE_TMR_ERROR_INVALID_TAG_TYPE                = 50331661,
    ECODE_TMR_ERROR_MULTIPLE_STATUS                 = 50331662,
    ECODE_TMR_ERROR_UNEXPECTED_TAG_ID               = 50331663,
    ECODE_TMR_ERROR_REGULATORY                      = 50331664,
    ECODE_TMR_ERROR_SYSTEM_RESOURCE                 = 50331665,

    /* =========================================================
     * LLRP COMMUNICATION
     * ========================================================= */
    ECODE_TMR_ERROR_LLRP_READER_CONNECTION_ALREADY_OPEN   = 67108865,
    ECODE_TMR_ERROR_LLRP_READER_CONNECTION_LOST_INTERNAL  = 67108866,
    ECODE_TMR_ERROR_LLRP_SENDIO_ERROR                     = 67108867,
    ECODE_TMR_ERROR_LLRP_RECEIVEIO_ERROR                  = 67108868,
    ECODE_TMR_ERROR_LLRP_RECEIVE_TIMEOUT                  = 67108869,
    ECODE_TMR_ERROR_LLRP_MSG_PARSE_ERROR                  = 67108870,
    ECODE_TMR_ERROR_LLRP_ALREADY_CONNECTED                = 67108871,
    ECODE_TMR_ERROR_LLRP_INVALID_RFMODE                   = 67108872,
    ECODE_TMR_ERROR_LLRP_UNDEFINED_VALUE                  = 67108873,
    ECODE_TMR_ERROR_LLRP_READER_ERROR                     = 67108874,
    ECODE_TMR_ERROR_LLRP_READER_CONNECTION_LOST            = 67108875,
    ECODE_TMR_ERROR_LLRP_CLIENT_CONNECTION_EXISTS          = 67108876

} TMR_ErrorCode;

/**
 * @brief Region 선택 정책 (오탈자 방지를 위해 enum으로 강제)
 */
typedef enum RFID_REGION {
    RFID_REGION_AUTO = 0, // 지원 목록 기반 자동 선택(정책: KR2 우선, 없으면 첫 항목)
    RFID_REGION_KR2, // KR2 (Korea)
    RFID_REGION_US, // US
    RFID_REGION_EU // EU
} RFID_REGION;

/**
 * @brief init()에 필요한 파라미터 묶음
 */
typedef struct rfid_init_params {
    int rfid_enable; // 0이면 RFID 기능 비활성
    const char *uri; // 예: "tmr:///dev/ttyUSB0"
    RFID_REGION region; // region 설정(AUTO 포함)
    const int *antennas; // 사용할 안테나 목록(예: {1,2})
    int antenna_count; // 안테나 개수
    int plan_timeout_ms; // read plan timeout(ms), 필요 시 0 허용
    int write_power_cdbm; // 송신 전력(cdBm), 필요 시 0 허용
} rfid_init_params_t;

/**
 * @brief 단일 태그 데이터(정렬 후 전체 리스트 반환 목적)
 */
typedef struct rfid_tag {
    char epc[RFID_EPC_MAX_LEN]; // RFID tag 값(EPC 문자열)
    int rssi; // 수신 강도(RSSI)
    uint32_t readcnt; // 읽힌 횟수(ReadCount)
    int antenna; // 수신 안테나 번호
    uint64_t ts; // 타임스탬프(ms/us 정책은 구현에서 정의)
} rfid_tag_t;

#ifdef __cplusplus
}
#endif

#endif  // RFID_TYPES_H_
