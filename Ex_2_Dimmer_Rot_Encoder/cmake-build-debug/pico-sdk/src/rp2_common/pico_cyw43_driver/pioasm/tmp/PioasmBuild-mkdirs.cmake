# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/amerilain/pico/pico-sdk/tools/pioasm"
  "/Users/amerilain/pico/Ex_2_Dimmer_Rot_Encoder/cmake-build-debug/pioasm"
  "/Users/amerilain/pico/Ex_2_Dimmer_Rot_Encoder/cmake-build-debug/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm"
  "/Users/amerilain/pico/Ex_2_Dimmer_Rot_Encoder/cmake-build-debug/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/tmp"
  "/Users/amerilain/pico/Ex_2_Dimmer_Rot_Encoder/cmake-build-debug/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/PioasmBuild-stamp"
  "/Users/amerilain/pico/Ex_2_Dimmer_Rot_Encoder/cmake-build-debug/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src"
  "/Users/amerilain/pico/Ex_2_Dimmer_Rot_Encoder/cmake-build-debug/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/PioasmBuild-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/amerilain/pico/Ex_2_Dimmer_Rot_Encoder/cmake-build-debug/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/PioasmBuild-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/amerilain/pico/Ex_2_Dimmer_Rot_Encoder/cmake-build-debug/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/PioasmBuild-stamp${cfgdir}") # cfgdir has leading slash
endif()
