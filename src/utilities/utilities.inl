// UtilityCore: A utility library. Part of the TAKUA Render project.
// Written by Yining Karl Li
// Version 0.5.13.39a_eulermod
//  
// File: utilities.inl
// A collection/kitchen sink of generally useful functions

#ifndef UTILITIES_INL
#define UTILITIES_INL

#ifdef __CUDACC__
#define HOST __host__
#define DEVICE __device__
#else
#define HOST
#define DEVICE
#endif
 
#include <iostream>
#include <sys/timeb.h>
#include <cstdio>
#include <cstring>
#include <fstream>
#include "../math/eigenmathutils.inl"

using namespace spatialmathCore;

//====================================
// Math stuff
//====================================

float utilityCore::clamp(float f, float min, float max){
    if(f<min){
        return min;
    }else if(f>max){
        return max;
    }else{
        return f;
    }
}

bool utilityCore::epsilonCheck(float a, float b){
    if(std::fabs(std::fabs(a)-std::fabs(b))<EPSILON){
        return true;
    }else{
        return false;
    }
}

//====================================
// String wrangling stuff
//====================================

bool utilityCore::replaceString(std::string& str, const std::string& from, const std::string& to) {
    size_t start_pos = str.find(from);
    if(start_pos == std::string::npos)
        return false;
    str.replace(start_pos, from.length(), to);
    return true;
}

std::string utilityCore::convertIntToString(int number){
    std::stringstream ss;
    ss << number;
    return ss.str();
}

std::vector<std::string> utilityCore::tokenizeString(std::string str, std::string separator){
    std::vector<std::string> results;
    char * cstr, *p;
    std::string strt = str;
    cstr = new char[strt.size()+1];
    std::strcpy (cstr, strt.c_str());
    p=std::strtok (cstr, separator.c_str());
    while (p!=NULL){
        results.push_back(p);
        p=strtok(NULL, separator.c_str());
    }
    delete [] cstr;
    delete [] p;
    return results;
}

std::vector<std::string> utilityCore::tokenizeStringByAllWhitespace(std::string str){
    std::stringstream strstr(str);
    std::istream_iterator<std::string> it(strstr);
    std::istream_iterator<std::string> end;
    std::vector<std::string> results(it, end);
    return results;
}

std::string utilityCore::getLastNCharactersOfString(std::string s, int n){
    return s.substr(s.length()-n, n);
}

std::string utilityCore::getFirstNCharactersOfString(std::string s, int n){
    return s.substr(0, n);
}

//====================================
// Time stuff
//====================================

int utilityCore::getMilliseconds(){
    timeb tb;
    ftime( &tb );
    int nCount = tb.millitm + (tb.time & 0xfffff) * 1000;
    return nCount;
}

int utilityCore::compareMilliseconds(int referenceTime){
    int elapsedMilliseconds = getMilliseconds() - referenceTime;
    if ( elapsedMilliseconds < 0 ){
        elapsedMilliseconds += 0x100000 * 1000;
    }
    return elapsedMilliseconds;
}

//====================================
// GL Stuff
//====================================

void utilityCore::fovToPerspective(float fovy, float aspect, float zNear, vec2& xBounds, 
                                   vec2& yBounds){
    yBounds[1] = zNear * tan(fovy*(float)PI/360.0f);
    yBounds[0] = -yBounds[1];
    xBounds[0] = yBounds[0]*aspect;
    xBounds[1] = yBounds[1]*aspect;
}

//====================================
// IO Stuff
//====================================

std::string utilityCore::readFileAsString(std::string filename){
    std::ifstream t(filename.c_str());
    std::string str;
    t.seekg(0, std::ios::end);   
    str.reserve(t.tellg());
    t.seekg(0, std::ios::beg);
    str.assign((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
    return str;
}

std::string utilityCore::getRelativePath(std::string path){
    std::string relativePath;
    std::vector<std::string> pathTokens = utilityCore::tokenizeString(path, "/");
    for(int i=0; i<pathTokens.size()-1; i++){
        relativePath = relativePath + pathTokens[i] + "/";
    }
    return relativePath;
}

#endif
