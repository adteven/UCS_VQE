#ifndef __SMD_H__
#define __SMD_H__
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#if defined(_WIN32) && !defined(__MINGW32__)
# ifndef SMDAPI
#  define SMDAPI __stdcall
# endif
#else
# ifndef SMDAPI
#  define SMDAPI
# endif
#endif


typedef void * smd_h;
#define SMD_INVALID_H	NULL

smd_h  smd_handle_create(void);
float  smd_handle_recv(smd_h handle, short *data, int sampleNum,int sampleRate);
float* smd_get_band_tonality(smd_h handle);
void  smd_handle_destory(smd_h handle);

#ifdef __cplusplus
}
#endif
#endif /* __SMD_H__ */

