/*
POSIX getopt for Windows

AT&T Public License

Code given out at the 1985 UNIFORUM conference in Dallas.

Signficant updates:
    2014-12-12: Updated to use CRL_HAVE_GETOPT preprocessor definition
*/

#ifdef CRL_HAVE_GETOPT
#include <getopt.h>
#else

#ifndef GETOPT_H_B9B6A349_4F17_4D0B_BBBC_F347117B4E27
#define GETOPT_H_B9B6A349_4F17_4D0B_BBBC_F347117B4E27

#ifdef __cplusplus
extern "C"
{
#endif

extern int opterr;
extern int optind;
extern int optopt;
extern char* optarg;

extern int getopt (int argc, char** argv, char* opts);

#ifdef __cplusplus
}
#endif

#endif // GETOPT_H_B9B6A349_4F17_4D0B_BBBC_F347117B4E27

#endif // CRL_HAVE_GETOPT
