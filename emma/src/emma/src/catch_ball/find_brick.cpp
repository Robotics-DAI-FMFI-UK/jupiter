#include <stdio.h>
#include <inttypes.h>


static int minr, maxr, mins, maxs;
uint8_t *buffer;
int sirka, vyska;

static uint8_t hlada_co;

static uint8_t FILL_R, FILL_G, FILL_B;

void hladacia_faza()
{
	FILL_R = 70;
	FILL_G = 255;
	FILL_B = 70;
	hlada_co = 1;
}

void zvyraznovacia_faza()
{
	FILL_R = 70;
	FILL_G = 70;
	FILL_B = 255;
	hlada_co = 2;
}

void zisti_rgb(int riadok, int stlpec, uint8_t *r, uint8_t *g, uint8_t *b)
{
  	      *b = buffer[riadok * sirka * 3 + stlpec * 3];
  	      *g = buffer[riadok * sirka * 3 + stlpec * 3 + 1];
  	      *r = buffer[riadok * sirka * 3 + stlpec * 3 + 2];
}

int spravna_farba2(int r, int g, int b)
{
  return (r > g * 2.5) && (r > b * 1.5) && (r > 100);
}

int spravna_farba (uint8_t r, uint8_t g, uint8_t b)
{
  if (hlada_co == 2)
  {
	  if ((g == 255) && (r == 70) && (b == 70)) return 1;
	  else return 0;
  }
  
  float h, s, v, max, min;
  h = 0;
 
  if ((uint16_t)r + (uint16_t)g + (uint16_t)b == 0) return 0;
   if (r < b){
    if(r < g){
      min = r;
    }
    else{
      min = g;
    }
  }
  else{
    if (b < g){
      min = b;
    }
    else{
      min = g;
    }
  }


  if (r > b){
    if(r > g){
      max = r;
      if (max!= min){
        h = 60*((g-b)/(max-min));
      }
    }
    else{
      max = g;
      if (max!= min){
        h = 60*(2+(b-r)/(max-min));
      }
    }
  }
  else{
    if (b > g){
      max = b;
      if (max!= min){
        h = 60*(4+(r-g)/(max - min));
      } 
    }
    else{
      max = g;
      if (max!= min){
        h = 60*(2+(b-r)/(max - min));
      }
    }
  }
 
  v = max;

  s = (max-min)/max;
  
  if (h < 0){
    h = h + 360;
  }
     
  return (( h > 355) || (h < 34)) && (s > 0.4) && (v > 80);
}

int fill(int riadok, int stlpec)
{
  static int fillcounter = 0;
  fillcounter++;

  //if (fillcounter % 1000 == 0) printf("fill(n=%d, r=%d, c=%d)\n", fillcounter, riadok, stlpec);

  if (riadok < minr) minr = riadok;
  if (riadok > maxr) maxr = riadok;
  if (stlpec < mins) mins = stlpec;
  if (stlpec > maxs) maxs = stlpec;
  
  buffer[riadok * sirka * 3 + stlpec * 3] = FILL_B;
  buffer[riadok * sirka * 3 + stlpec * 3 + 1] = FILL_G;
  buffer[riadok * sirka * 3 + stlpec * 3 + 2] = FILL_R;
  
  uint8_t r, g, b;
  
  zisti_rgb(riadok, stlpec + 1, &r, &g, &b);
  int kolko = 1;
  
  if (spravna_farba(r, g, b))
    kolko += fill(riadok, stlpec + 1);

  zisti_rgb(riadok, stlpec - 1, &r, &g, &b);
  
  if (spravna_farba(r, g, b))
    kolko += fill(riadok, stlpec - 1);

  zisti_rgb(riadok - 1, stlpec, &r, &g, &b);
    
  if (spravna_farba(r, g, b))
    kolko += fill(riadok - 1, stlpec);

  zisti_rgb(riadok + 1, stlpec, &r, &g, &b);
  
  if (spravna_farba(r, g, b))
    kolko += fill(riadok + 1, stlpec);

  return kolko;
}
 
void najdi_kocku(int *sirka_kocky, int *vyska_kocky, int *velkost_kocky, int *riadok, int *stlpec)
{
      const uint8_t *p = buffer;
      for (int i = 0; i < vyska; i++)
      {
         // lavy okraj
         buffer[i*sirka*3] = 0;
         buffer[i*sirka*3 + 1] = 0;
         buffer[i*sirka*3 + 2] = 0;
         
         // pravy okraj
         buffer[(i + 1)*sirka*3 - 3] = 0;
         buffer[(i + 1)*sirka*3 - 2] = 0;
         buffer[(i + 1)*sirka*3 - 1] = 0;
      }
      
      int index_zaciatku_dolneho_riadku = (vyska - 1) * sirka * 3;
      for (int i = 0; i < sirka; i++)
      {
         // horny okraj
         buffer[i*3] = 0;
         buffer[i*3 + 1] = 0;
         buffer[i*3 + 2] = 0;
         
         // dolny okraj
         buffer[index_zaciatku_dolneho_riadku + i * 3] = 0;
         buffer[index_zaciatku_dolneho_riadku + i * 3 + 1] = 0;
         buffer[index_zaciatku_dolneho_riadku + i * 3 + 2] = 0;
      }      

      int doteraz_najvacsi = 0;
      int doteraz_najv_sirka = 0;
      int doteraz_najv_vyska = 0;
      int doteraz_najv_riadok = 0;
      int doteraz_najv_stlpec = 0;
      
      hladacia_faza();
      
      for (int i = 0; i < vyska; i++)
        for (int j = 0; j < sirka; j++)
        {
  	      uint8_t b = *(p++);
  	      uint8_t g = *(p++);
  	      uint8_t r = *(p++);

  	      if (spravna_farba(r, g, b))
  	      {
                  mins = sirka, minr = vyska, maxs = -1, maxr = -1;
                  //printf("call fill\n");
                  int pocet = fill(i, j);
                  
                  if ((pocet > doteraz_najvacsi) && (pocet >= 1000))
                  {
                      doteraz_najvacsi = pocet;
                      doteraz_najv_sirka = maxs - mins + 1;
                      doteraz_najv_vyska = maxr - minr + 1;
                      doteraz_najv_riadok = (maxr + minr) / 2;
                      doteraz_najv_stlpec = (maxs + mins) / 2;
                  }
  	      }
        }
    //  printf("velkost: %d, sirka: %d, vyska: %d\n", doteraz_najvacsi, 
    //           doteraz_najv_sirka, doteraz_najv_vyska);

      *sirka_kocky = doteraz_najv_sirka;
      *vyska_kocky = doteraz_najv_vyska;
      *velkost_kocky = doteraz_najvacsi;
      *riadok = doteraz_najv_riadok;
      *stlpec = doteraz_najv_stlpec;
      zvyraznovacia_faza();
      fill(*riadok, *stlpec);
}
