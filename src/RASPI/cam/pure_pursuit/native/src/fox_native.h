#ifndef FOX_NATIVE_H
#define FOX_NATIVE_H
#include <stddef.h>
#include <stdint.h>
#if defined(__FAST_MATH__)
#error "fox_native: prohibido -ffast-math"
#endif
#if defined(_WIN32)
#define FOX_API __declspec(dllexport)
#else
#define FOX_API __attribute__((visibility("default")))
#endif
#endif
