#pragma once
#include <string>
#include <filesystem>
class Common
{
public:
	Common();
	~Common();


	static std::string trim(const std::string& str);
	static std::wstring trim(const std::wstring& str);
	static std::wstring toWideStr(const std::string& str);
	static std::string toStr(const std::wstring& str);
	static std::wstring getAbsoluteFilePath(const std::wstring& path);
	static void trimAll(std::wstring &s);
};

