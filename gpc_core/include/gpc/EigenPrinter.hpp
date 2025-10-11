#pragma once
#include <Eigen/Dense>
#include <sstream>
#include <string>
#include <iostream>

class EigenPrinter
{
public:
    enum class Style {
        SingleLine,   // all elements in one line
        Pretty,       // multiline, aligned columns
        Csv           // CSV compatible
    };

    EigenPrinter(Style style = Style::SingleLine,
                 int precision = Eigen::StreamPrecision,
                 const std::string& name = "")
        : mStyle(style), mPrecision(precision), mName(name) {}

    // print to std::cout directly
    template<typename Derived>
    void print(const Eigen::MatrixBase<Derived>& mat) const {
        std::cout << toString(mat) << std::endl;
    }

    // convert to string (so you can log it)
    template<typename Derived>
    std::string toString(const Eigen::MatrixBase<Derived>& mat) const {
        std::ostringstream oss;
        if (!mName.empty())
            oss << mName << " = ";

        switch (mStyle)
        {
            case Style::SingleLine:
                oss << mat.format(singleLineFmt());
                break;
            case Style::Pretty:
                oss << "\n" << mat.format(prettyFmt());
                break;
            case Style::Csv:
                oss << mat.format(csvFmt());
                break;
        }
        return oss.str();
    }

    void setName(const std::string& name) { mName = name; }

private:
    Style mStyle;
    int mPrecision;
    std::string mName;

    Eigen::IOFormat singleLineFmt() const {
        return Eigen::IOFormat(mPrecision, 0, ", ", "; ", "", "", "[", "]");
    }

    Eigen::IOFormat prettyFmt() const {
        return Eigen::IOFormat(mPrecision, 0, ", ", "\n", "[", "]", "[", "]");
    }

    Eigen::IOFormat csvFmt() const {
        return Eigen::IOFormat(mPrecision, 0, ",", "\n", "", "", "", "");
    }
};
