//
// Created by previato on 9/26/17.
//

#ifndef ROBOT2_VREPEXCEPTION_H
#define ROBOT2_VREPEXCEPTION_H

#include <exception>
#include <string>

class VRepException : public std::exception {
private:
    std::string str;
public:
    explicit VRepException(const char* str) {
            this->str = "VRepException: " + *str;
    }

    const char* what() const throw() override {
        return str.c_str();
    }
};


#endif //ROBOT2_VREPEXCEPTION_H
