#ifndef GORILLA_PRECISION_H
#define GORILLA_PRECISION_H

namespace gorilla {
#ifndef SINGLE_PRECISION
    typedef double real;
#else
    typedef float real;
#endif
}

#endif // GORILLA_PRECISION_H