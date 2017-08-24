#ifndef __UCS_TRACE_H__
#define __UCS_TRACE_H__

#include "webrtc/common_types.h"
using namespace webrtc;

typedef void(*ucs_trace_cb)(const char* message, int length);

class UcsTrace : public TraceCallback
{
public:
    void Print(TraceLevel level, const char* message, int length);
    void RegisterTraceCb(ucs_trace_cb cb);
private:
    ucs_trace_cb trace_cb;
};
#endif // __UCS_TRACE_H__
