/* Needed for SWIG_exception */
%include "exception.i"

/* Input typemap: convert the input object ($input) to IplImage */
%typemap(in) IplImage * (IplImage header){
	void * vptr;
    // if $input wraps a CvMat, extract it and convert to IplImage
	if( SWIG_ConvertPtr($input, (&vptr), $descriptor( CvMat * ), 0) != -1 ){
	    $1 = cvGetImage((CvMat *)vptr, &header);
    }
    // if $input wraps an IplImage, extract it
	else if( SWIG_ConvertPtr($input, (&vptr), $descriptor( IplImage * ), 0) != -1 ){
        $1 = (IplImage *) vptr;
    }
    // illegal input
    else {
		SWIG_exception( SWIG_TypeError, "%%typemap(in) IplImage * : could not convert input object to IplImage");
		SWIG_fail;
	}
}


/* Typecheck typemap: determine if input object can be converted to IplImage */
%typemap(typecheck) IplImage * {
	void * vptr;
	if(SWIG_ConvertPtr($input, (void **) &vptr, $descriptor(CvMat *), 0) != -1 ){
		$1 = 1;
	}
	else if(SWIG_ConvertPtr($input, (void **) &vptr, $1_descriptor, 0) != -1 ) {
		$1 = 1;
	}
	else{
		$1 = 0;
	}
}
