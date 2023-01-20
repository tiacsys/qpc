// AUTOGENERATED FILE. DO NOT EDIT.
#ifndef _MOCKSPI_H
#define _MOCKSPI_H

#include "unity.h"
#include "Spi.h"

// Ignore the following warnings, since we are copying code
#if defined(__GNUC__) && !defined(__ICC) && !defined(__TMS470__)
#if __GNUC__ > 4 || (__GNUC__ == 4 && (__GNUC_MINOR__ > 6 || (__GNUC_MINOR__ == 6 && __GNUC_PATCHLEVEL__ > 0)))
#pragma GCC diagnostic push
#endif
#if !defined(__clang__)
#pragma GCC diagnostic ignored "-Wpragmas"
#endif
#pragma GCC diagnostic ignored "-Wunknown-pragmas"
#pragma GCC diagnostic ignored "-Wduplicate-decl-specifier"
#endif

#ifdef __cplusplus
extern "C" {
#endif

void MockSpi_Init(void);
void MockSpi_Destroy(void);
void MockSpi_Verify(void);




#define SPI_beginTransaction_Expect() SPI_beginTransaction_CMockExpect(__LINE__)
void SPI_beginTransaction_CMockExpect(UNITY_LINE_TYPE cmock_line);
#define SPI_transfer_Expect(data) SPI_transfer_CMockExpect(__LINE__, data)
void SPI_transfer_CMockExpect(UNITY_LINE_TYPE cmock_line, uint32_t data);
#define SPI_endTransaction_Expect() SPI_endTransaction_CMockExpect(__LINE__)
void SPI_endTransaction_CMockExpect(UNITY_LINE_TYPE cmock_line);

#ifdef __cplusplus
}
#endif

#if defined(__GNUC__) && !defined(__ICC) && !defined(__TMS470__)
#if __GNUC__ > 4 || (__GNUC__ == 4 && (__GNUC_MINOR__ > 6 || (__GNUC_MINOR__ == 6 && __GNUC_PATCHLEVEL__ > 0)))
#pragma GCC diagnostic pop
#endif
#endif

#endif
