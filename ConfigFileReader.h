#pragma once
#include <string>
#include <map>
#include <vector>
class ConfigFileReader
{
public:
	ConfigFileReader(const std::string& configFileName);
	ConfigFileReader(const std::wstring& configFileName);
	ConfigFileReader(const std::string& configFileName, const char& cListDelimiter);
	ConfigFileReader(const std::wstring& configFileName, const wchar_t& cListDelimiter);

	bool open();
	bool getValue(const std::string& key, std::string& value);
	bool getValue(const std::string& key, double& value);
	bool getValue(const std::string& key, int& value);
	
	bool getValue(const std::wstring& key, std::wstring& value);
	bool getValue(const std::wstring& key, double& value);
	bool getValue(const std::wstring& key, int& value);
	bool getValue(const std::wstring& key, bool& value);

	bool getValues(const std::wstring& key, std::vector<double>& vValues);
	bool getValues(const std::wstring& key, std::vector<std::wstring>& vValues);
	
	bool getValues(const std::string& key, std::vector<double>& vValues);
	


	~ConfigFileReader();

private:
	bool									m_bValid;
	std::wstring							m_sFileName;
	std::map<std::wstring, std::wstring>	m_mDictionary;
	wchar_t									m_cListDelimiter;
};

