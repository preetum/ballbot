/* Blob.i */
%module Blob

/* Hide warning messages ******************************************************/

/* Blob.h:59: Warning(362): operator= ignored */
%warnfilter(362) operator=;

/* Blob.h:51: Warning(509): Overloaded CBlob(CBlob const *) is shadowed by 
              CBlob(CBlob const &) at Blob.h:50. */
%warnfilter(509) CBlob(CBlob const *);

/* Blob.h:194: Warning(503): Can't wrap 'operator COperadorBlob*' unless 
               renamed to a valid identifier */
%ignore COperadorBlob;

/* Blob.h:167: Warning(312): Nested class not currently supported (ignored) */
#pragma SWIG nowarn=312;

/*****************************************************************************/

%{
	#include "Blob.h"
%}

%include "helpers.i"
%include "Blob.h"

