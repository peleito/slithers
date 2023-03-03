//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: main.cpp
//
// MATLAB Coder version            : 5.5
// C/C++ source code generated on  : 16-Feb-2023 15:04:13
//

/*************************************************************************/
/* This automatically generated example C++ main file shows how to call  */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

// Include Files
#include "main.h"
#include "generate_helix_cpp.h"
#include "generate_helix_cpp_terminate.h"
#include "rt_nonfinite.h"
#include "coder_array.h"

// Function Declarations
static double argInit_real_T();

// Function Definitions
//
// Arguments    : void
// Return Type  : double
//
static double argInit_real_T()
{
  return 0.0;
}

//
// Arguments    : int argc
//                char **argv
// Return Type  : int
//
int main(int, char **)
{
  // The initialize function is being called automatically from your entry-point
  // function. So, a call to initialize is not included here. Invoke the
  // entry-point functions.
  // You can call entry-point functions multiple times.
  main_generate_helix_cpp();
  // Terminate the application.
  // You do not need to do this more than one time.
  generate_helix_cpp_terminate();
  return 0;
}

//
// Arguments    : void
// Return Type  : void
//
void main_generate_helix_cpp()
{
  coder::array<double, 3U> poses;
  double a_tmp;
  // Initialize function 'generate_helix_cpp' input arguments.
  a_tmp = argInit_real_T();
  // Call the entry-point 'generate_helix_cpp'.
  generate_helix_cpp(a_tmp, a_tmp, a_tmp, a_tmp, poses);
}

//
// File trailer for main.cpp
//
// [EOF]
//
