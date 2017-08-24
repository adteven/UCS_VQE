// audio3A.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <stdlib.h>
#include <stdio.h>

#include "ucs_vqe.h"
#include "audio3A.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// The one and only application object

CWinApp theApp;

using namespace std;

int _tmain(int argc, TCHAR* argv[], TCHAR* envp[])
{
	int nRetCode = 0;

	HMODULE hModule = ::GetModuleHandle(NULL);

	if (hModule != NULL)
	{
		// initialize MFC and print and error on failure
		if (!AfxWinInit(hModule, NULL, ::GetCommandLine(), 0))
		{
			// TODO: change error code to suit your needs
			_tprintf(_T("Fatal Error: MFC initialization failed\n"));
			nRetCode = 1;
		}
		else
		{
			// TODO: code your application's behavior here.
            audio_main();
		}
	}
	else
	{
		// TODO: change error code to suit your needs
		_tprintf(_T("Fatal Error: GetModuleHandle failed\n"));
		nRetCode = 1;
	}

	return nRetCode;
}

static void log_output(const char* message, int length)
{
    printf("UCS_TRACE: %s\n", message);
}

void audio_main()
{
    const short kFrameLen = 80;
    FILE* farfp = NULL;
    FILE* nearfp = NULL;
    FILE* outfp = NULL;
    short farend[kFrameLen] = { 0 };
    short neerend[kFrameLen] = { 0 };
    short output[kFrameLen] = { 0 };
    UcsVqeConfig config;

    farfp = fopen("E:\\svn\\UCS_VQE\\test\\far2.pcm", "rb");
    nearfp = fopen("E:\\svn\\UCS_VQE\\test\\near2.pcm", "rb");
    outfp = fopen("E:\\svn\\UCS_VQE\\test\\aec_out.pcm", "wb");

    if (!farfp || !nearfp || !outfp)
    {
        printf("open test file failed.\n");
        getchar();
        return;
    }

    printf("proc begin\n");
    memset(&config, 0x00, sizeof(UcsVqeConfig));
    config.aec_enable = ucs_true;
    config.agc_enable = ucs_true;
    config.ns_enable = ucs_true;
    if (UCSVQE_Init(kUcsSampleRate8kHz, &config) != 0)
    {
        return;
    }
    UCSVQE_SetLogCb(log_output);

    while ((kFrameLen == fread(farend, 2, kFrameLen, farfp))
        && (kFrameLen == fread(neerend, 2, kFrameLen, nearfp)))
    {
        UCSVQE_FarendAnalysis(farend);
        UCSVQE_Process(neerend, 0, output);

        fwrite(output, 2, kFrameLen, outfp);
    }
    UCSVQE_Closed();
    printf("proc end\n");
    getchar();
}