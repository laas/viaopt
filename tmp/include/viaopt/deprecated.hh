// Copyright (C) 2010 Florent Lamiraux, Thomas Moulard, JRL, CNRS/AIST.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef VIAOPT_DEPRECATED_HH
# define VIAOPT_DEPRECATED_HH

// Define a suffix which can be used to tag a type, a function or a a
// variable as deprecated (i.e. it will emit a warning when using it).
//
// Tagging a function as deprecated:
//  void foo () VIAOPT_DEPRECATED;
//
// Tagging a type as deprecated:
//  class Foo {};
//  typedef Foo Bar VIAOPT_DEPRECATED;
//
// Tagging a variable as deprecated:
//  int a VIAOPT_DEPRECATED = 0;
//
// The use of a macro is required as this is /not/ a standardized
// feature of C++ language or preprocessor, even if most of the
// compilers support it.
# ifdef __GNUC__
#  define VIAOPT_DEPRECATED __attribute__ ((deprecated))
# else
#  ifdef _MSC_VER
#   define VIAOPT_DEPRECATED __declspec (deprecated)
#  else
// If the compiler is not recognized, drop the feature.
#   define VIAOPT_DEPRECATED /* nothing */
#  endif // __MSVC__
# endif // __GNUC__

#endif //! VIAOPT_DEPRECATED_HH
