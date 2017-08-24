#include "ucs_trace.h"

void UcsTrace::Print(TraceLevel level, const char* message, int length)
{
    if (trace_cb)
    {
        trace_cb(message, length);
    }
}

void UcsTrace::RegisterTraceCb(ucs_trace_cb cb)
{
    trace_cb = cb;
}