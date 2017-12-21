// TestClient_Cpp.cpp : 콘솔 응용 프로그램에 대한 진입점을 정의합니다.
//

#include "stdafx.h"
using namespace std;

void showError(const char * msg);

int main()
{
	WSADATA data;
	::WSAStartup(MAKEWORD(2, 2), &data);

	SOCKET client = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

	if (client == INVALID_SOCKET)
		showError("Failed to generate client");

	sockaddr_in addr = { 0 };

	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = inet_addr("192.168.1.34");
	addr.sin_port = htons(22999);

	if (connect(client, (sockaddr *)&addr, sizeof(addr)) == SOCKET_ERROR)
		showError("Failed to connection");
	
	send(client, "ConnectionOK", 12, 0);
	cout << "completely connected" << endl;

	int n = 0;
	while (1)
	{
		while (n != 1) cin >> n;
		while (n == 1)
		{
			send(client, "19775824 10 0 1 0", 17, 0);
			n = 0;
		}
	}
	
	closesocket(client);
	::WSACleanup();
	return 0;
}

void showError(const char * msg)
{
	cout << "에러 : " << msg << endl;
	exit(-1);
}