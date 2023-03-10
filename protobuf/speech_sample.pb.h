/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.5 */

#ifndef PB_PROTOBUF_SPEECH_SAMPLE_PB_H_INCLUDED
#define PB_PROTOBUF_SPEECH_SAMPLE_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Struct definitions */
typedef struct _SpeechSample { 
    /* blub */
    uint32_t timestamp; 
    /* bla */
    uint16_t pkts_encoded; 
    /* audio samples array, max_count is FRAME_SIZE, see hostcom_task.c
 TODO: Obviously, include respective C header to avoid hard-coding. */
    uint16_t samples[128]; 
} SpeechSample;


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define SpeechSample_init_default                {0, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}
#define SpeechSample_init_zero                   {0, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}

/* Field tags (for use in manual encoding/decoding) */
#define SpeechSample_timestamp_tag               1
#define SpeechSample_pkts_encoded_tag            2
#define SpeechSample_samples_tag                 3

/* Struct field encoding specification for nanopb */
#define SpeechSample_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, UINT32,   timestamp,         1) \
X(a, STATIC,   REQUIRED, UINT32,   pkts_encoded,      2) \
X(a, STATIC,   FIXARRAY, UINT32,   samples,           3)
#define SpeechSample_CALLBACK NULL
#define SpeechSample_DEFAULT NULL

extern const pb_msgdesc_t SpeechSample_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define SpeechSample_fields &SpeechSample_msg

/* Maximum encoded size of messages (where known) */
#define SpeechSample_size                        522

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
