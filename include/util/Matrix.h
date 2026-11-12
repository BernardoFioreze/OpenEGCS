/*
util/Matrix.h

Simple 2D jagged container (vector of vectors) used by ElevatorSystem to
store the customers waiting at each floor. Originally split across
matrix.h/matrix.cpp with the implementation file `#include`d directly by
ElevatorSystem.h (a fragile pattern - templates cannot be split across
translation units without explicit instantiation). Since this is a
template class, declaration and implementation are kept together in a
single header, which is the correct/idiomatic way to write header-only
C++ templates.

Adapted from the original liftsim `matrix` class (2011, Nivvedan).
*/

#ifndef LIFTSIM_UTIL_MATRIX_H
#define LIFTSIM_UTIL_MATRIX_H

#include <vector>

namespace openEGCS {
namespace util {

template <class Obj>
class Matrix {

	public:
		Matrix(int l = 0, int b = 0) : mat(l) {
			if ((l < 0) || (b < 0)) {
				l = 0;
				b = 0;
			}
			len = l;
			wid = b;
			for (int i = 0; i < l; i++) {
				mat[i].resize(b);
			}
		}

		~Matrix() = default;

		const Obj& at(int x, int y) const {						// Returns the object at (x,y)
			return mat.at(x).at(y);
		}

		const std::vector<Obj>& at(int x) const {					// Returns the vector of objects at x
			return mat.at(x);
		}

		Obj& at(int x, int y) {										// Sets the object at (x,y)
			return mat.at(x).at(y);
		}

		std::vector<Obj>& at(int x) {								// Sets the vector of objects at y
			return mat.at(x);
		}

		const std::vector<Obj>& operator[] (int row) const {		// As usual
			return mat.at(row);
		}

		std::vector<Obj>& operator[] (int row) {					// As usual
			return mat.at(row);
		}

		int length() const {										// Returns length of matrix
			return len;
		}

		int width() const {										// Returns width of matrix
			return wid;
		}

		void resize(int size) {									// Resizes the matrix
			mat.resize(size);
		}

	private:
		int len;
		int wid;
		std::vector<std::vector<Obj>> mat;
};

} // namespace util
} // namespace openEGCS

#endif
