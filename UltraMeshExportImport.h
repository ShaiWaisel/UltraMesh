#pragma once
#ifdef ULTRAMESH_EXPORTS
#define ULTRAMESH_API __declspec(dllexport)
#else
#define ULTRAMESH_API __declspec(dllimport)
#endif
