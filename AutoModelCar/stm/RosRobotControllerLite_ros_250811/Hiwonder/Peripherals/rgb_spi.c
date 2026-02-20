
#include "rgb_spi.h"
#include "global.h"
#include "lwmem_porting.h"
#include "iwdg.h"

static uint8_t pixelBuffer[Pixel_S1_NUM+1][24]; //灯珠
//uint8_t *pixelBuffer;

//static uint32_t  rgb_pixel[Pixel_S1_NUM];

void rgb_SendArray(void){
//    memset(pixelBuffer[2] , 0xFF , 24);
    HAL_IWDG_Refresh(&hiwdg);
	HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)pixelBuffer, (Pixel_S1_NUM+1)*24);
    HAL_IWDG_Refresh(&hiwdg);
//    for(int i = 0 ; i < Pixel_S1_NUM ; i++){
//        HAL_SPI_Transmit(&hspi1, (uint8_t*)pixelBuffer, 48 , 0xFFFF); // (Pixel_S1_NUM)*24
////        int k = 0;
//    }
}

void WS2812b_Configuration(void)
{
//    pixelBuffer = LWMEM_CCM_MALLOC(sizeof(uint8_t)*Pixel_S1_NUM*24);
    HAL_IWDG_Refresh(&hiwdg);
    memset(pixelBuffer , 0x00 , Pixel_S1_NUM*24); //CODE0
    HAL_IWDG_Refresh(&hiwdg);
    osDelay(10);
    memset(pixelBuffer[Pixel_S1_NUM] , 0x00 , 24);
    HAL_IWDG_Refresh(&hiwdg);
//    osDelay(100);
    rgb_SendArray();
    for(int i = 0 ; i < 2; i++){
        HAL_IWDG_Refresh(&hiwdg);
        osDelay(10);
    }
    memset(pixelBuffer , CODE0 , Pixel_S1_NUM*24); //
//    osDelay(10);
}

//void set_id_rgb_color(uint8_t id , uint8_t* rgb)
//{
//    if(id >= Pixel_S1_NUM) //RGB灯编号为 0 、 1
//    {
//        return;
//    }
//    int i = 0;
//    for(i=0;i<=7;i++){
//		pixelBuffer[id][i]= ( (rgb[1] & (1 << (7 -i)) )? (CODE1):CODE0 );
//	}
//    for(i=8;i<=15;i++){
//		pixelBuffer[id][i]= ( (rgb[0] & (1 << (15-i)) )? (CODE1):CODE0 );
//	}
//    for(i=16;i<=23;i++){
//		pixelBuffer[id][i]= ( (rgb[2] & (1 << (23-i)) )? (CODE1):CODE0 );
//	}
//    rgb_SendArray();
//}


void set_id_rgb_color(struct Pixel* rgb)
{
    if(rgb->pixel_index >= Pixel_S1_NUM) //RGB灯编号为 1\2
    {
        return;
    }
    int i = 0;
    for(i=0;i<=7;i++){
		pixelBuffer[rgb->pixel_index-1][i]= ( (rgb->g & (1 << (7 -i)) )? (CODE1):CODE0 );
	}
    for(i=8;i<=15;i++){
		pixelBuffer[rgb->pixel_index-1][i]= ( (rgb->r & (1 << (15-i)) )? (CODE1):CODE0 );
	}
    for(i=16;i<=23;i++){
		pixelBuffer[rgb->pixel_index-1][i]= ( (rgb->b & (1 << (23-i)) )? (CODE1):CODE0 );
	}
    rgb_SendArray();
}

void set_rgb_color(int num , struct Pixel* rgb)
{
    int i = 0;
    for(int n = 0 ; n < num ; n++)
    {
        if(rgb[n].pixel_index <= Pixel_S1_NUM)
        {
            for(i=0;i<=7;i++){
                pixelBuffer[rgb[n].pixel_index][i]= ( (rgb[n].g & (1 << (7 -i)) )? (CODE1):CODE0 );
            }
            for(i=8;i<=15;i++){
                pixelBuffer[rgb[n].pixel_index][i]= ( (rgb[n].r & (1 << (15-i)) )? (CODE1):CODE0 );
            }
            for(i=16;i<=23;i++){
                pixelBuffer[rgb[n].pixel_index][i]= ( (rgb[n].b & (1 << (23-i)) )? (CODE1):CODE0 );
            }
        }
    }

//    for(i=0;i<=7;i++){
//		pixelBuffer[0][i]= ( (rgb[0].g & (1 << (7 -i)) )? (CODE1):CODE0 );
//	}
//    for(i=8;i<=15;i++){
//		pixelBuffer[0][i]= ( (rgb[0].r & (1 << (15-i)) )? (CODE1):CODE0 );
//	}
//    for(i=16;i<=23;i++){
//		pixelBuffer[0][i]= ( (rgb[0].b & (1 << (23-i)) )? (CODE1):CODE0 );
//	}
//    for(i=0;i<=7;i++){
//		pixelBuffer[1][i]= ( (rgb[1] & (1 << (7 -i)) )? (CODE1):CODE0 );
//	}
//    for(i=8;i<=15;i++){
//		pixelBuffer[1][i]= ( (rgb[1] & (1 << (15-i)) )? (CODE1):CODE0 );
//	}
//    for(i=16;i<=23;i++){
//		pixelBuffer[1][i]= ( (rgb[1] & (1 << (23-i)) )? (CODE1):CODE0 );
//	}
    rgb_SendArray();
}

//void set_rgb_color(uint8_t* rgb)
//{
//    int i = 0;
//    for(i=0;i<=7;i++){
//		pixelBuffer[0][i]= ( (rgb[1] & (1 << (7 -i)) )? (CODE1):CODE0 );
//	}
//    for(i=8;i<=15;i++){
//		pixelBuffer[0][i]= ( (rgb[0] & (1 << (15-i)) )? (CODE1):CODE0 );
//	}
//    for(i=16;i<=23;i++){
//		pixelBuffer[0][i]= ( (rgb[2] & (1 << (23-i)) )? (CODE1):CODE0 );
//	}
//    for(i=0;i<=7;i++){
//		pixelBuffer[1][i]= ( (rgb[4] & (1 << (7 -i)) )? (CODE1):CODE0 );
//	}
//    for(i=8;i<=15;i++){
//		pixelBuffer[1][i]= ( (rgb[3] & (1 << (15-i)) )? (CODE1):CODE0 );
//	}
//    for(i=16;i<=23;i++){
//		pixelBuffer[1][i]= ( (rgb[5] & (1 << (23-i)) )? (CODE1):CODE0 );
//	}
//    rgb_SendArray();
//}
