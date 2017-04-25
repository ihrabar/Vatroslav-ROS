
namespace Vatroslav
{
	#define CONST const
	#define VOID  void

	typedef char CHAR;
	typedef unsigned long DWORD;
	typedef wchar_t WCHAR;
	typedef void *PVOID;
	typedef unsigned short WORD;
	typedef unsigned char BYTE;
	typedef PVOID HANDLE;
	typedef CONST WCHAR *LPCWSTR;
	typedef __nullterminated CONST CHAR *LPCSTR;

	#ifdef UNICODE
	 typedef LPCWSTR LPCTSTR; 
	#else
	 typedef LPCSTR LPCTSTR;
	#endif

}