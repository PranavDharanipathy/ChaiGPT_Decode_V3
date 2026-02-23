package com.cppoptimizations;

public class NativeLib {

    // Used to load the 'cppoptimizations' library on application startup.
    static {
        System.loadLibrary("cppoptimizations");
    }

    /**
     * A native method that is implemented by the 'cppoptimizations' native library,
     * which is packaged with this application.
     */
    public native String stringFromJNI();
}