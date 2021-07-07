#include "ConfigFileReader.h"
#include <fstream>
#include "Common.h"
#include <sstream>
#include <regex>

ConfigFileReader::ConfigFileReader(const std::string& configFileName)
	:ConfigFileReader(Common::toWideStr(configFileName))
{
}


ConfigFileReader::ConfigFileReader(const std::wstring& configFileName)
	:ConfigFileReader(configFileName, L',')
{
}


ConfigFileReader::ConfigFileReader(const std::string& configFileName, const char& cListDelimiter)
	: ConfigFileReader(Common::toWideStr(configFileName), (wchar_t)cListDelimiter)
{
}

ConfigFileReader::ConfigFileReader(const std::wstring& configFileName, const wchar_t& cListDelimiter)
{
	m_sFileName = configFileName;
	m_bValid = false;
	m_cListDelimiter = cListDelimiter;
}


ConfigFileReader::~ConfigFileReader()
{
}


bool ConfigFileReader::open()
{
	std::wifstream ifs(m_sFileName);
	if (!ifs.good())
	{
		wprintf_s(L"Cannot open config file");
		return false;
	}

	while (!ifs.eof())
	{
		std::wstring line;
		std::getline(ifs, line);
		size_t indexOfRemark = line.find_first_of(L"%");
		if ((indexOfRemark > 0) && (indexOfRemark < line.size()))
		line = line.substr(0, indexOfRemark - 1);
		size_t lastTab = line.find_last_of('\t');
		while ((lastTab == line.size()-1) && (line.size() > 0))
		{
			line = line.substr(0,  lastTab);
			lastTab = line.find_last_of('\t');
		}
		size_t lastSpace = line.find_last_of(' ');
		while ((lastSpace == line.size() - 1) && (line.size() > 0))
		{
			line = line.substr(0, lastSpace);
			lastSpace = line.find_last_of(' ');
		}

		size_t indexOfEqualSign = line.find_first_of(L"=");
		if (std::string::npos == indexOfEqualSign || indexOfEqualSign == line.length() - 1 || indexOfEqualSign==0)
			continue;

		std::wstring key = Common::trim(line.substr(0,indexOfEqualSign));
		std::wstring val = line.substr(indexOfEqualSign+1);
		while ((val.size() > 0) && val[0] == ' ')
			val = val.substr(1, val.size() - 1);
		while ((val.size() > 0) && val[0] == '\t')
			val = val.substr(1, val.size() - 1);

		if (key.empty() || val.empty())
			continue;

		m_mDictionary[key] = val;
// 		printf("%s = %s\n", key.c_str(), val.c_str());
	}

	return true;
}


bool ConfigFileReader::getValue(const std::string& key, std::string& value)
{
	std::wstring sWideVal;
	bool bGot = getValue(Common::toWideStr(key), sWideVal);
	if (bGot)
	{
		value = Common::toStr(sWideVal);
	}
	return bGot;
}

bool ConfigFileReader::getValue(const std::string& key, double& value)
{
	return getValue(Common::toWideStr(key), value);
}

bool ConfigFileReader::getValue(const std::string& key, int& value)
{
	return getValue(Common::toWideStr(key), value);
}



bool ConfigFileReader::getValue(const std::wstring& key, std::wstring& value)
{
	if (m_mDictionary.cend() == m_mDictionary.find(key))
		return false;

	value = m_mDictionary[key];
	return true;
}

bool ConfigFileReader::getValue(const std::wstring& key, double& value)
{
	std::wstring sVal;
	bool got = getValue(key, sVal);
	if (!got)
		return false;
	value = _wtof(sVal.c_str());
	return true;
}

bool ConfigFileReader::getValue(const std::wstring& key, int& value)
{
	std::wstring sVal;
	bool got = getValue(key, sVal);
	if (!got)
		return false;
	value = _wtoi(sVal.c_str());
	return true;
}

bool ConfigFileReader::getValue(const std::wstring& key, bool& value)
{
	std::wstring sVal;
	bool got = getValue(key, sVal);
	if (!got)
		return false;
	std::wsmatch wmatch;
	std::wregex pattern(L"\\s*true\\s*", std::regex_constants::ECMAScript | std::regex_constants::icase);
	value = std::regex_match(sVal.cbegin(), sVal.cend(), wmatch, pattern);
	return true;
}

bool ConfigFileReader::getValues(const std::wstring& key, std::vector<std::wstring>& vValues)
{
	std::wstring sVal;
	bool got = getValue(key, sVal);
	if (!got)
		return false;

	vValues.clear();
	//split
	std::wstringstream ss(sVal);
	std::wstring item;
	while (std::getline(ss, item,m_cListDelimiter)) 
	{
		vValues.push_back(item);
	}
	return true;
}

bool ConfigFileReader::getValues(const std::wstring& key, std::vector<double>& vValues)
{
	std::vector<std::wstring> vStrValues;
	bool got = getValues(key, vStrValues);
	if (!got)
		return false;

	vValues.clear();
	for (std::wstring& iter : vStrValues)
		vValues.push_back(_wtof(iter.c_str()));
	
	return true;

}

bool ConfigFileReader::getValues(const std::string& key, std::vector<double>& vValues)
{
	return getValues(Common::toWideStr(key), vValues);
}
