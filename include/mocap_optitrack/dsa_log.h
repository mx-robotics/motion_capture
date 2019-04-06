#ifndef __DSA_LOG_H__
#define __DSA_LOG_H__

#define DSA_WARN(...) printf(__VA_ARGS__); printf("\n")
#define DSA_INFO(...) printf(__VA_ARGS__); printf("\n")
#define DSA_DEBUG(...) printf(__VA_ARGS__); printf("\n")
#define DSA_INFO_ONCE(...) printf(__VA_ARGS__); printf("\n")

#endif /*__DSA_LOG_H__*/
