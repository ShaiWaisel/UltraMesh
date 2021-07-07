#include "Common.h"
#include <string>
#include <algorithm> 
#include <cctype>
#include <locale>
Common::Common()
{
}


Common::~Common()
{
}

// trim from start (in place)
static inline void ltrim(std::wstring &s) {
	s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int ch) {
		return !std::isspace(ch);
	}));
}

// trim from end (in place)
static inline void rtrim(std::wstring &s) {
	s.erase(std::find_if(s.rbegin(), s.rend(), [](int ch) {
		return !std::isspace(ch);
	}).base(), s.end());
}

// trim from both ends (in place)
void Common::trimAll(std::wstring &s) {
	ltrim(s);
	rtrim(s);
}

std::string Common::trim(const std::string& str)
{
	size_t first = str.find_first_not_of(' ');
	size_t last = str.find_last_not_of(' ');
	if (first == std::string::npos || last == std::string::npos)
		return "";
	return str.substr(first, (last - first + 1));
}

std::wstring Common::trim(const std::wstring& str)
{
	size_t first = str.find_first_not_of(' ');
	size_t last = str.find_last_not_of(' ');
	if (first == std::wstring::npos || last == std::wstring::npos)
		return L"";
	return str.substr(first, (last - first + 1));
}

std::wstring Common::toWideStr(const std::string& str)
{
	return std::wstring(str.cbegin(), str.cend());
}

std::string Common::toStr(const std::wstring& wstr)
{
	return std::string(wstr.cbegin(), wstr.cend());
}

std::wstring
Common::getAbsoluteFilePath(const std::wstring& path)
{
	// check if network folder
	if ((wcscmp(path.c_str(), L"\\\\") == 0) ||
		(wcscmp(path.c_str(), L"//") == 0))
	{
		return path;
	}
	else
	{
		return std::experimental::filesystem::system_complete(path);
	}
}

