// UtilityCore: A utility library. Part of the TAKUA Render project.
// Written by Yining Karl Li
// Version 0.5.13.39a_eulermod
//  
// File: utilities.h
// A collection/kitchen sink of generally useful functions

#ifndef UTILITIES_H
#define UTILITIES_H

#ifdef __CUDACC__
#define HOST __host__
#define DEVICE __device__
#else
#define HOST
#define DEVICE
#endif

#include <algorithm>
#include <istream>
#include <ostream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>
#include "../math/eigenmathutils.inl"

using namespace spatialmathCore;

//====================================
// Useful Math Constants
//====================================
#define PI                        3.1415926535897932384626422832795028841971
#define TWO_PI                    6.2831853071795864769252867665590057683943
#define SQRT_OF_ONE_THIRD         0.5773502691896257645091487805019574556476
#define E                         2.7182818284590452353602874713526624977572
#define EPSILON                   0.000000001
#define ZERO_ABSORPTION_EPSILON   0.00001
#define RAY_BIAS_AMOUNT           0.0002
#define REALLY_BIG_NUMBER         1000000000000000000

namespace utilityCore {

//====================================
// Function Declarations
//====================================

//Math stuff
extern inline float clamp(float f, float min, float max);
extern inline bool epsilonCheck(float a, float b);

//String wrangling stuff
extern inline bool replaceString(std::string& str, const std::string& from, const std::string& to);
extern inline std::vector<std::string> tokenizeString(std::string str, std::string separator); 
extern inline std::vector<std::string> tokenizeStringByAllWhitespace(std::string str);
extern inline std::string convertIntToString(int number);
extern inline std::string getLastNCharactersOfString(std::string s, int n);
extern inline std::string getFirstNCharactersOfString(std::string s, int n);

//Time stuff
extern inline int getMilliseconds();
extern inline int compareMilliseconds(int referenceTime);

//Useful stuff for GL
extern inline void fovToPerspective(float fovy, float aspect, float zNear, evec2& xBounds, 
									evec2& yBounds);

//IO Stuff
extern inline std::string readFileAsString(std::string filename);
extern inline std::string getRelativePath(std::string path);

}

#include "utilities.inl"

#endif
