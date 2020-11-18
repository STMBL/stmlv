/*
* This file is part of the stmbl project.
*
* Copyright (C) 2013-2015 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2013-2015 Nico Stute <crinq@crinq.de>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <string.h>
#include "hal.h"
#include "commands.h"
#include "stm32f3xx_hal.h"
#include "config.h"

char config[4 * 1024];
const char *config_ro = (char *)0x08002000;

extern config_template_t config_templates[];
extern const uint32_t num_of_config_templates;

void flashloadconf(char *ptr) {
  strncpy(config, config_ro, sizeof(config));
}
COMMAND("flashloadconf", flashloadconf, "load config from flash");

void flashsaveconf(char *ptr) {
  printf("erasing flash page...\n");
  HAL_FLASH_Unlock();
  // FLASH_Unlock();
  uint32_t PageError = 0;
  HAL_StatusTypeDef status;
  FLASH_EraseInitTypeDef eraseinitstruct;
  eraseinitstruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  eraseinitstruct.PageAddress = (uint32_t)config_ro;
  eraseinitstruct.NbPages     = 4;
  status                      = HAL_FLASHEx_Erase(&eraseinitstruct, &PageError);
  if(status != HAL_OK) {
  // if(FLASH_EraseSector(FLASH_Sector_2, VoltageRange_3) != FLASH_COMPLETE) {
    printf("error!\n");
    HAL_FLASH_Lock();
    // FLASH_Lock();
    return;
  }
  printf("saving conf\n");
  int i   = 0;
  int ret = 0;
  for(int i = 0; i < sizeof(config); i += 4){
    uint32_t word = (config[i + 0]) + (config[i + 1] << 8) + (config[i + 2] << 16) + (config[i + 3] << 24);
    ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)config_ro + i, word) != HAL_OK;
    // ret = FLASH_ProgramByte((uint32_t)(config_ro + i), config[i]) != FLASH_COMPLETE;
    if(ret) {
      printf("error writing %i\n", ret);
      break;
    }
  }
  printf("OK %i bytes written\n", i);
  HAL_FLASH_Lock();
  // FLASH_Lock();
}
COMMAND("flashsaveconf", flashsaveconf, "save config to flash");

void loadconf(char *ptr) {
  hal_parse(config);
}
COMMAND("loadconf", loadconf, "parse config");

void showconf(char *ptr) {
  printf("%s", config_ro);
}
COMMAND("showconf", showconf, "show config");

void appendconf(char *ptr) {
  printf("adding %s\n", ptr);
  strncat(config, ptr, sizeof(config) - 1);
  strncat(config, "\n", sizeof(config) - 1);
}
COMMAND("appendconf", appendconf, "append string to config");

void deleteconf(char *ptr) {
  config[0] = '\0';
}
COMMAND("deleteconf", deleteconf, "delete config");

// void hardboot(char *ptr) {
//   printf("erasing flash page...\n");
//   HAL_FLASH_Unlock();
//   // FLASH_Unlock();
//   if(FLASH_EraseSector(FLASH_Sector_4, VoltageRange_3) != FLASH_COMPLETE) {
//     printf("error!\n");
//     FLASH_Lock();
//     return;
//   }
//   printf("OK, call bootloader\n");
//   HAL_FLASH_Lock();
//   // FLASH_Lock();
//   NVIC_SystemReset();
// }
