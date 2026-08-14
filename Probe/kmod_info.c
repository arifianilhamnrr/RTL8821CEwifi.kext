#include <mach/kmod.h>
#include <mach/mach_types.h>

extern kern_return_t _start(kmod_info_t *, void *);
extern kern_return_t _stop(kmod_info_t *, void *);

KMOD_EXPLICIT_DECL(com.crangel.RTL8821CEProbe, "0.32.0", _start, _stop)

static kern_return_t probe_start(kmod_info_t *ki, void *data)
{
    (void)ki;
    (void)data;
    return KERN_SUCCESS;
}

static kern_return_t probe_stop(kmod_info_t *ki, void *data)
{
    (void)ki;
    (void)data;
    return KERN_SUCCESS;
}

kmod_start_func_t *_realmain = probe_start;
kmod_stop_func_t *_antimain = probe_stop;
