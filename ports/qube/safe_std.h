//============================================================================
// SafeQP/C Real-Time Event Framework (RTEF)
//
// Copyright (C) 2005 Quantum Leaps, LLC. All rights reserved.
//
//                    Q u a n t u m  L e a P s
//                    ------------------------
//                    Modern Embedded Software
//
// SPDX-License-Identifier: LicenseRef-QL-commercial
//
// This software is licensed under the terms of the Quantum Leaps commercial
// licenses. Please contact Quantum Leaps for more information about the
// available licensing options.
//
// RESTRICTIONS
// You may NOT :
// (a) redistribute, encumber, sell, rent, lease, sublicense, or otherwise
//     transfer rights in this software,
// (b) remove or alter any trademark, logo, copyright or other proprietary
//     notices, legends, symbols or labels present in this software,
// (c) plagiarize this software to sidestep the licensing obligations.
//
// Quantum Leaps contact information:
// <www.state-machine.com/licensing>
// <info@state-machine.com>
//============================================================================
#ifndef SAFE_STD_H_
#define SAFE_STD_H_

#include <stdio.h>
#include <string.h>

// portable "safe" facilities from <stdio.h> and <string.h> ................
#ifdef _WIN32 // Windows OS?

#define MEMMOVE_S(dest_, num_, src_, count_) \
    memmove_s(dest_, num_, src_, count_)

#define STRNCPY_S(dest_, destsiz_, src_) \
    strncpy_s(dest_, destsiz_, src_, _TRUNCATE)

#define STRCAT_S(dest_, destsiz_, src_) \
    strcat_s(dest_, destsiz_, src_)

#define SNPRINTF_S(buf_, bufsiz_, format_, ...) \
    _snprintf_s(buf_, bufsiz_, _TRUNCATE, format_, __VA_ARGS__)

#define PRINTF_S(format_, ...) \
    printf_s(format_, __VA_ARGS__)

#define FPRINTF_S(fp_, format_, ...) \
    fprintf_s(fp_, format_, __VA_ARGS__)

#ifdef _MSC_VER
#define FREAD_S(buf_, bufsiz_, elsiz_, count_, fp_) \
    fread_s(buf_, bufsiz_, elsiz_, count_, fp_)
#else
#define FREAD_S(buf_, bufsiz_, elsiz_, count_, fp_) \
    fread(buf_, elsiz_, count_, fp_)
#endif // _MSC_VER

#define FOPEN_S(fp_, fName_, mode_) \
if (fopen_s(&fp_, fName_, mode_) != 0) { \
    fp_ = (FILE *)0; \
} else (void)0

#define LOCALTIME_S(tm_, time_) \
    localtime_s(tm_, time_)

#else // other OS (Linux, MacOS, etc.) .....................................

#define MEMMOVE_S(dest_, num_, src_, count_) \
    memmove(dest_, src_, count_)

#define STRNCPY_S(dest_, destsiz_, src_) do { \
    strncpy(dest_, src_, destsiz_);           \
    dest_[(destsiz_) - 1] = '\0';             \
} while (false)

#define STRCAT_S(dest_, destsiz_, src_) \
    strcat(dest_, src_)

#define SNPRINTF_S(buf_, bufsiz_, format_, ...) \
    snprintf(buf_, bufsiz_, format_, __VA_ARGS__)

#define PRINTF_S(format_, ...) \
    printf(format_, __VA_ARGS__)

#define FPRINTF_S(fp_, format_, ...) \
    fprintf(fp_, format_, __VA_ARGS__)

#define FREAD_S(buf_, bufsiz_, elsiz_, count_, fp_) \
    fread(buf_, elsiz_, count_, fp_)

#define FOPEN_S(fp_, fName_, mode_) \
    (fp_ = fopen(fName_, mode_))

#define LOCALTIME_S(tm_, time_) \
    memcpy(tm_, localtime(time_), sizeof(struct tm))

#endif // _WIN32

#endif // SAFE_STD_H_
