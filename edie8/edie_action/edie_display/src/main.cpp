/*This source code copyrighted by Lazy Foo' Productions 2004-2024
and may not be redistributed without written permission.*/

//Using SDL and standard IO
//#include <SDL.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include <GL/glew.h>
#include <stdio.h>
#include <string>

using namespace std;

//Screen dimension constants
const int SCREEN_WIDTH = 1080;//640;
const int SCREEN_HEIGHT = 1080;//480;

enum KeyPressSurfaces
{
	KEY_PRESS_SURFACE_DEFAULT = 0,
	KEY_PRESS_SURFACE_UP,
	KEY_PRESS_SURFACE_DOWN,
	KEY_PRESS_SURFACE_LEFT,
	KEY_PRESS_SURFACE_RIGHT,
	KEY_PRESS_SURFACE_TOTAL
};

bool Init();															// SDL을 시작하고 윈도우를 만듦
bool LoadMedia();													// 미디어를 로드함
void Close();															// 미디어를 해체하고 SDL을 종료함
SDL_Surface* LoadSurface( string path );	// 개별 이미지를 로드함

SDL_Window* g_window = NULL;																	// 랜더링될 창
SDL_Surface* g_screen_surface = NULL;													// 창에 포함될 표면
SDL_Surface* g_key_press_surfaces[ KEY_PRESS_SURFACE_TOTAL ];	// 키를 누르면 표시할 이미지
SDL_Surface* g_current_surface = NULL;												// 현재 표시중인 이미지


// 메인 함수
int main( int argc, char* args[] )
{
	//The window we'll be rendering to
	SDL_Window* window = NULL;
	
	//The surface contained by the window
	SDL_Surface* screen_surface = NULL;
	Init();

	//Initialize SDL
	if( SDL_Init( SDL_INIT_VIDEO ) < 0 )
	{
		printf( "SDL could not initialize! SDL_Error: %s\n", SDL_GetError() );
	}
	else
	{
		//Create window
		window = SDL_CreateWindow( "SDL Tutorial", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN );
		if( window == NULL )
		{
			printf( "Window could not be created! SDL_Error: %s\n", SDL_GetError() );
		}
		else
		{
			//Get window surface
			screen_surface = SDL_GetWindowSurface( window );

			//Fill the surface white
			SDL_FillRect( screen_surface, NULL, SDL_MapRGB( screen_surface->format, 0x00, 0x00, 0x00 ) );
			
			//Hack to get window to stay up
			SDL_Event e; 
			bool quit = false; 
			while( !quit )
			{
				// SDL 큐 이벤트 처리하기. 
				while( SDL_PollEvent( &e ) )
				{
					if( e.type == SDL_QUIT )
					{
						printf( "SDL_QUIT received\n" );
						quit = true;
					}
					else if( e.type == SDL_KEYDOWN )
					{
						switch( e.key.keysym.sym )
						{
							case SDLK_UP:
								g_current_surface = g_key_press_surfaces[ KEY_PRESS_SURFACE_UP ];
								printf( "UP key pressed\n" );
								break;
							case SDLK_DOWN:
								g_current_surface = g_key_press_surfaces[ KEY_PRESS_SURFACE_DOWN ];
								printf( "DOWN key pressed\n" );
								break;
							case SDLK_LEFT:
								g_current_surface = g_key_press_surfaces[ KEY_PRESS_SURFACE_LEFT ];
								printf( "LEFT key pressed\n" );
								break;
							case SDLK_RIGHT:
								g_current_surface = g_key_press_surfaces[ KEY_PRESS_SURFACE_RIGHT ];
								printf( "RIGHT key pressed\n" );
								break;
							default:
								g_current_surface = g_key_press_surfaces[ KEY_PRESS_SURFACE_DEFAULT ];
								printf( "DEFAULT key pressed\n" );
								break;
						}
					}
				}
				SDL_BlitSurface( g_current_surface, NULL, screen_surface, NULL);
				//Update the surface
				SDL_UpdateWindowSurface( window );
			}
		}
	}

	//Destroy window
	SDL_DestroyWindow( window );

	//Quit SDL subsystems
	SDL_Quit();

	return 0;
}


// SDL을 시작하고 윈도우를 만듦
bool Init()
{
	LoadMedia();
	return true;
}

// 미디어를 로드함
bool LoadMedia()
{
	bool is_success = true;

	string path = "image/";
	string file_names[ KEY_PRESS_SURFACE_TOTAL ] =
	{
		"000.jpg",
		"001.jpg",
		"002.jpg",
		"003.jpg",
		"004.jpg"
	};	// 순서 중요. enum 순서와 맞춰야 함. 

	for( int i = 0; i < KEY_PRESS_SURFACE_TOTAL; ++i )
	{
		g_key_press_surfaces[i] = IMG_Load( (path + file_names[i]).c_str() );
		if( g_key_press_surfaces[i] == NULL )
		{
			printf( "Failed to load image %s!\n", file_names[i].c_str() );
			is_success = false;
		}
	}

	return is_success;
}

// 미디어를 해체하고 SDL을 종료함
void Close()
{

}

// 개별 이미지를 로드함
SDL_Surface* LoadSurface( string path )
{
	SDL_Surface* loaded_surface = SDL_LoadBMP( path.c_str() );
	if( loaded_surface == NULL )
	{
		printf( "Unable to load image %s! SDL Error: %s\n", path.c_str(), SDL_GetError() );
	}
	// else
	// {
	// 	//Convert surface to screen format
	// 	SDL_Surface* optimized_surface = SDL_ConvertSurface( loaded_surface, g_screen_surface->format, NULL );
	// 	if( optimized_surface == NULL )
	// 	{
	// 		printf( "Unable to optimize image %s! SDL Error: %s\n", path.c_str(), SDL_GetError() );
	// 	}

	// 	//Get rid of old loaded surface
	// 	SDL_FreeSurface( loaded_surface );

	// 	return optimized_surface;
	// }
	return loaded_surface;
}

