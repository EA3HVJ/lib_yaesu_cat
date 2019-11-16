#ifndef DEBUG_H
#define DEBUG_H

#ifdef __cplusplus
extern "C"
{
#endif

/**************
 * DEBUG MODE *
 **************/

#ifdef _DEBUG
#define DEBUG_STR "<DEBUG> "
#define DEBUG_PRINT(x, ...) printf(DEBUG_STR x"\n", ## __VA_ARGS__)
#define DEBUG_PLACE printf(DEBUG_STR "FILE: \"%s\" · LINE: %d · %s()\n", __FILE__, __LINE__, __func__)
#define DPRINTF(x,...) printf(x, ## __VA_ARGS__)
#else
#define DEBUG_PRINT(x, ...)
#define DEBUG_PLACE ;
#define DPRINTF(x, ...)
#endif


#ifdef __cplusplus
}
#endif

#endif /* DEBUG_H */

