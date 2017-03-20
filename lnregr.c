/**
  Linear regression
  John Brooks (c) 2014
  http://jwbrooks.blogspot.ro/2014/02/arduino-linear-regression-function.html
*/

/**
  Simple linear regression
  The ab array is comprised of the slope=ab[0] and intercept=ab[1].

  @param x array
  @param y array
  @param ab linear regression coefficients
  @param n length of the x and y arrays
*/
void lnRegr(float* x, float* y, float* ab, int n) {
  // Initialize the variables
  float xbar = 0;
  float ybar = 0;
  float xybar = 0;
  float xsqbar = 0;
  float xy = 0;
  float xx = 0;
  float dx = 0;
  float dy = 0;

  // Calculations required for linear regression
  for (int i = 0; i < n; i++) {
    xbar += x[i];
    ybar += y[i];
    //xybar += x[i] * y[i];
    //xsqbar += x[i] * x[i];
  }
  xbar = xbar / n;
  ybar = ybar / n;
  //xybar = xybar / n;
  //xsqbar = xsqbar / n;

  for (int i = 0; i < n; i++) {
    dx = x[i] - xbar;
    dy = y[i] - ybar;
    xy += dx * dy;
    xx += dx * dx;
  }
  
  // Simple linear regression algorithm
  //ab[0] = (xybar - xbar * ybar) / (xsqbar - xbar * xbar);
  //ab[1] = ybar - ab[0] * xbar;
  ab[0] = xy / xx;
  ab[1] = ybar - ab[0] * xbar;
}
