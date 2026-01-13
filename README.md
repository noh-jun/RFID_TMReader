# RFID Wrapper Project

## 개요
본 프로젝트는 **ThingMagic MercuryAPI C API**를 기반으로 Linux 환경에서 사용할 수 있는 래퍼(wrapper) 라이브러리와 테스트 프로그램을 제공합니다.

구성 요소:

- C99 기반 **C 래퍼 라이브러리** (공유 라이브러리, `.so`)
- C99 + C++17 기반 **C++ 래퍼 라이브러리** (공유 라이브러리, `.so`)
- C 래퍼 테스트 프로그램
- C++ 래퍼 테스트 프로그램

원본 MercuryAPI SDK는 `third_party` 경로에 그대로 유지되며, 프로젝트 코드는 이를 직접 수정하지 않고 래핑하는 구조로 설계되었습니다.

---

## 개발 환경

- OS: Ubuntu 24.04
- IDE: JetBrains CLion
- Build System: CMake
- Language Standard
    - C Wrapper: C99
    - C++ Wrapper: C++17

---

## 프로젝트 구조

```
.
├── c_only
├── cpp_only
├── c_test_only
├── cpp_test_only
├── include
├── src
├── third_party
│   └── mercuryapi
│       ├── api
│       └── c
├── CMakeLists.txt
└── README.md
```

### 디렉터리 설명

| 디렉터리 | 설명 |
|---------|------|
| `c_only` | C99 기반 C 래퍼 라이브러리 (`.so`) 생성용 CMake 프로젝트 |
| `cpp_only` | C99 + C++17 기반 C++ 래퍼 라이브러리 (`.so`) 생성용 CMake 프로젝트 |
| `c_test_only` | `c_only` 래퍼 테스트 프로젝트 |
| `cpp_test_only` | `cpp_only` 래퍼 테스트 프로젝트 |
| `include` | `cpp_only`에서 사용하는 C++ 래퍼 클래스 헤더 |
| `src` | `cpp_only`에서 사용하는 C++ 래퍼 구현 소스 |
| `third_party/mercuryapi/api` | C 래퍼 인터페이스 계층 |
| `third_party/mercuryapi/c` | ThingMagic MercuryAPI C API 원본 SDK |

---

## CMake 빌드 구조

본 프로젝트는 **루트 CMakeLists.txt의 옵션(option)으로 서브 타깃을 1개만 선택하여 빌드하는 구조**를 사용합니다.

루트 `CMakeLists.txt`에는 다음 옵션이 있으며, **네 개 중 정확히 하나만 `ON`**이어야 합니다.

```cmake
option(BUILD_C_ONLY     "Build mercuryapi only" OFF)
option(BUILD_CPP_ONLY   "Build mercuryapi_cpp only" OFF)
option(BUILD_C_TEST     "Build C test executable only" OFF)
option(BUILD_CPP_TEST   "Build C++ test executable only" ON)
```

- 동시에 2개 이상을 `ON`으로 설정하면 **예외 처리로 CMake 단계에서 빌드가 중단**되도록 되어 있습니다.
- 빌드 타깃 변경은 **CMake 캐시를 갱신(리로드/리컨피그)** 해야 정확히 반영됩니다.

---

## 빌드 절차 (중요)

빌드 타깃 옵션을 변경한 경우, 아래 순서를 반드시 지켜야 합니다.

1. CMake Cache Reload (또는 re-configure)
2. Re-build
3. (필요 시) Install 명령 실행

> **각 테스트 코드는 install 경로에서 library를 참조**하도록 설계되었습니다.  
> `install-debug/`, `install-release/`는 **빌드 시 자동 생성되지 않습니다.**  
> 터미널에서 **`cmake --build {build paths} --target install`** 명령을 실행해야 생성됩니다.  

---

## 빌드 방법

### 공통 규칙: 옵션은 1개만 ON

아래 4개 중 **하나만** `ON`으로 지정하세요.

- `-DBUILD_C_ONLY=ON`
- `-DBUILD_CPP_ONLY=ON`
- `-DBUILD_C_TEST=ON`
- `-DBUILD_CPP_TEST=ON`

---

### CLion에서 빌드하는 경우

CLion을 사용하더라도 **옵션은 CMake profile의 CMake options에 지정**해야 합니다.

- 예: `-DBUILD_CPP_TEST=ON`

빌드 타깃을 바꿨다면 반드시 다음을 수행하세요.

1) **Reload CMake Project** (캐시 갱신)

2) **Build**

> Install은 CLion 메뉴가 아니라, 위의 터미널 명령(`cmake --install ...`)으로 수행하는 것을 권장합니다.

---

## Install 결과물

| 디렉터리 | 설명 |
|---------|------|
| `install-debug` | Debug 빌드 결과물 |
| `install-release` | Release 빌드 결과물 |

해당 디렉터리에는 다음 항목들이 포함됩니다.

- 공유 라이브러리 (`.so`)
- 헤더 파일
- CMake 패키지 설정 파일

---

## 테스트 실행

테스트 프로그램은 **CLion의 Run / Debug 기능**을 사용하여 실행합니다.

테스트 대상 프로젝트:

- `c_test_only`
- `cpp_test_only`

별도의 커맨드라인 실행 방법은 제공하지 않습니다.

---

## 참고 사항

- MercuryAPI 원본 SDK는 Window 의존성을 직접 수정했습니다.
- 모든 래퍼 로직은 third_party 외부에서 관리됩니다.
- 프로젝트는 유지보수성과 확장성을 고려하여 설계되었습니다.
