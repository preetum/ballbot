/* BlobResult.i */
%module BlobResult

/* Hide warnings **************************************************************/

/* BlobResult.h:98: Warning(362): operator= ignored */
%warnfilter(362) operator=;

/* BlobResult.h:133: Warning(509): Overloaded GetBlob(int) is shadowed by 
                     GetBlob(int) const at BlobResult.h:132. */
%warnfilter(509) GetBlob(int);

/*****************************************************************************/

%{
	#include "BlobResult.h"
%}
%include "helpers.i"
%include "BlobResult.h"
