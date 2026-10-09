cmake_minimum_required(VERSION 3.10)
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  if(DEFINED ENV{VITASDK})
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VITASDK}/share/vita.toolchain.cmake" CACHE PATH "toolchain file")
  else()
    message(FATAL_ERROR "Please define VITASDK")
  endif()
endif()

project(BounceEvolution C CXX ASM)
include("${VITASDK}/share/vita.cmake" REQUIRED)

set(VITA_APP_NAME "Bounce Evolution")
set(VITA_TITLEID  "BNCE00001")
set(VITA_VERSION  "01.00")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2 -g")
add_executable(${PROJECT_NAME}
  vita_port.c
  ${CMAKE_CURRENT_SOURCE_DIR}/bounce_reloc.o
)


target_link_libraries(${PROJECT_NAME}
  vitaGL
  vitashark
  SceShaccCgExt
  taihen_stub
  SceShaccCg_stub
  mathneon
  png
  z
  stdc++
  pthread
  m
  SceAudio_stub
  SceCtrl_stub
  SceTouch_stub
  SceMotion_stub
  SceGxm_stub
  SceDisplay_stub
  SceCommonDialog_stub
  SceSysmodule_stub
  SceAppMgr_stub
  SceKernelDmacMgr_stub
  ScePower_stub
)

vita_create_self(eboot.bin ${PROJECT_NAME} UNSAFE)
vita_create_vpk(${PROJECT_NAME}.vpk ${VITA_TITLEID} eboot.bin
  VERSION ${VITA_VERSION}
  NAME ${VITA_APP_NAME}
  FILE sce_sys/icon0.png sce_sys/icon0.png
       sce_sys/pic0.png sce_sys/pic0.png
       sce_sys/livearea/contents/bg0.png sce_sys/livearea/contents/bg0.png
       sce_sys/livearea/contents/startup.png sce_sys/livearea/contents/startup.png
       sce_sys/livearea/contents/template.xml sce_sys/livearea/contents/template.xml
)
