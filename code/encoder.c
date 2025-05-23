#include "encoder.h"
int16 Encoder_speed_l=0;
int16 Encoder_speed_r=0;

void encoder_filter(void)
{
  Encoder_speed_l=-encoder_get_count(ENCODER_l);
  Encoder_speed_r=encoder_get_count(ENCODER_r);

  encoder_clear_count(ENCODER_l);
  encoder_clear_count(ENCODER_r);

}

void encoder_init(void)
{

    encoder_dir_init(ENCODER_l , ENCODER_l_A  , ENCODER_l_B );
    encoder_dir_init(ENCODER_r , ENCODER_r_A ,  ENCODER_r_B );

}
