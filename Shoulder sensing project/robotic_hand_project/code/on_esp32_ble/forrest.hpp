#ifndef RF_PREDICT
#define RF_PREDICT
void rf_predict(float *in, int *cls_hist){
#define NUM_TREES 20
#define NUM_CLASSES 3
#define NUM_FEATURES 600
	// tree 0
    if(in[160] <= 0.68) {
        if(in[221] <= -0.47) {
            if(in[67] <= 9.70) {
                cls_hist[0]++;
            }
            else {
                if(in[234] <= 0.55) {
                    if(in[395] <= -0.07) {
                        cls_hist[0]++;
                    }
                    else {
                        cls_hist[1]++;
                    }
                }
                else {
                    cls_hist[0]++;
                }
            }
        }
        else {
            if(in[385] <= 13.38) {
                if(in[124] <= 1.60) {
                    cls_hist[0]++;
                }
                else {
                    if(in[22] <= -0.26) {
                        cls_hist[0]++;
                    }
                    else {
                        cls_hist[2]++;
                    }
                }
            }
            else {
                cls_hist[1]++;
            }
        }
    }
    else {
        if(in[115] <= 12.77) {
            if(in[106] <= 1.59) {
                if(in[168] <= -0.78) {
                    if(in[58] <= 0.19) {
                        cls_hist[0]++;
                    }
                    else {
                        cls_hist[2]++;
                    }
                }
                else {
                    cls_hist[0]++;
                }
            }
            else {
                if(in[317] <= -0.78) {
                    cls_hist[0]++;
                }
                else {
                    cls_hist[2]++;
                }
            }
        }
        else {
            if(in[244] <= 0.14) {
                cls_hist[1]++;
            }
            else {
                cls_hist[0]++;
            }
        }
    }
    
	// tree 1
    if(in[85] <= 10.00) {
        if(in[100] <= 0.96) {
            if(in[361] <= 15.77) {
                if(in[121] <= 15.40) {
                    if(in[270] <= -1.53) {
                        cls_hist[0]++;
                    }
                    else {
                        cls_hist[0]++;
                    }
                }
                else {
                    cls_hist[1]++;
                }
            }
            else {
                cls_hist[1]++;
            }
        }
        else {
            if(in[334] <= -0.65) {
                if(in[77] <= 0.17) {
                    cls_hist[2]++;
                }
                else {
                    cls_hist[0]++;
                }
            }
            else {
                cls_hist[0]++;
            }
        }
    }
    else {
        if(in[151] <= 10.47) {
            if(in[174] <= -1.26) {
                if(in[556] <= -0.32) {
                    cls_hist[0]++;
                }
                else {
                    if(in[50] <= 0.46) {
                        cls_hist[0]++;
                    }
                    else {
                        cls_hist[2]++;
                    }
                }
            }
            else {
                if(in[247] <= 8.39) {
                    if(in[357] <= 0.24) {
                        cls_hist[0]++;
                    }
                    else {
                        cls_hist[1]++;
                    }
                }
                else {
                    cls_hist[0]++;
                }
            }
        }
        else {
            if(in[235] <= 8.37) {
                if(in[122] <= -0.32) {
                    cls_hist[0]++;
                }
                else {
                    cls_hist[1]++;
                }
            }
            else {
                cls_hist[0]++;
            }
        }
    }
    
	// tree 2
    if(in[373] <= 11.46) {
        if(in[94] <= 0.90) {
            if(in[415] <= 12.21) {
                if(in[319] <= 13.73) {
                    if(in[159] <= -0.90) {
                        cls_hist[0]++;
                    }
                    else {
                        cls_hist[0]++;
                    }
                }
                else {
                    cls_hist[1]++;
                }
            }
            else {
                cls_hist[1]++;
            }
        }
        else {
            if(in[340] <= -0.91) {
                if(in[366] <= 2.77) {
                    cls_hist[2]++;
                }
                else {
                    cls_hist[0]++;
                }
            }
            else {
                if(in[293] <= -0.40) {
                    if(in[179] <= 0.03) {
                        cls_hist[0]++;
                    }
                    else {
                        cls_hist[2]++;
                    }
                }
                else {
                    cls_hist[0]++;
                }
            }
        }
    }
    else {
        if(in[127] <= 10.23) {
            if(in[267] <= -0.15) {
                cls_hist[2]++;
            }
            else {
                cls_hist[0]++;
            }
        }
        else {
            cls_hist[1]++;
        }
    }
    
	// tree 3
    if(in[159] <= -0.83) {
        if(in[322] <= -0.59) {
            cls_hist[1]++;
        }
        else {
            cls_hist[0]++;
        }
    }
    else {
        if(in[274] <= -1.61) {
            if(in[216] <= 0.88) {
                cls_hist[2]++;
            }
            else {
                cls_hist[0]++;
            }
        }
        else {
            if(in[433] <= 12.37) {
                if(in[391] <= 13.28) {
                    if(in[322] <= -1.41) {
                        cls_hist[0]++;
                    }
                    else {
                        cls_hist[0]++;
                    }
                }
                else {
                    cls_hist[1]++;
                }
            }
            else {
                cls_hist[1]++;
            }
        }
    }
    
	// tree 4
    if(in[427] <= 12.06) {
        if(in[328] <= -1.39) {
            if(in[112] <= 0.74) {
                if(in[309] <= 0.53) {
                    cls_hist[0]++;
                }
                else {
                    if(in[573] <= -0.74) {
                        cls_hist[0]++;
                    }
                    else {
                        cls_hist[1]++;
                    }
                }
            }
            else {
                if(in[282] <= 1.27) {
                    cls_hist[2]++;
                }
                else {
                    cls_hist[0]++;
                }
            }
        }
        else {
            if(in[403] <= 12.87) {
                if(in[246] <= -0.17) {
                    if(in[127] <= 14.00) {
                        cls_hist[0]++;
                    }
                    else {
                        cls_hist[1]++;
                    }
                }
                else {
                    if(in[183] <= -0.84) {
                        cls_hist[0]++;
                    }
                    else {
                        cls_hist[0]++;
                    }
                }
            }
            else {
                cls_hist[1]++;
            }
        }
    }
    else {
        if(in[127] <= 7.91) {
            cls_hist[0]++;
        }
        else {
            cls_hist[1]++;
        }
    }
    
	// tree 5
    if(in[397] <= 12.41) {
        if(in[142] <= 1.20) {
            if(in[103] <= 11.84) {
                if(in[451] <= 12.29) {
                    if(in[82] <= 0.87) {
                        cls_hist[0]++;
                    }
                    else {
                        cls_hist[0]++;
                    }
                }
                else {
                    cls_hist[1]++;
                }
            }
            else {
                cls_hist[1]++;
            }
        }
        else {
            if(in[126] <= 0.78) {
                if(in[322] <= -0.62) {
                    cls_hist[2]++;
                }
                else {
                    cls_hist[0]++;
                }
            }
            else {
                if(in[165] <= -0.98) {
                    cls_hist[1]++;
                }
                else {
                    cls_hist[0]++;
                }
            }
        }
    }
    else {
        cls_hist[1]++;
    }
    
	// tree 6
    if(in[100] <= 1.27) {
        if(in[228] <= -0.00) {
            if(in[211] <= 7.59) {
                if(in[19] <= 9.37) {
                    cls_hist[0]++;
                }
                else {
                    cls_hist[1]++;
                }
            }
            else {
                if(in[129] <= -0.87) {
                    if(in[367] <= 9.68) {
                        cls_hist[0]++;
                    }
                    else {
                        cls_hist[2]++;
                    }
                }
                else {
                    if(in[292] <= -1.49) {
                        cls_hist[2]++;
                    }
                    else {
                        cls_hist[0]++;
                    }
                }
            }
        }
        else {
            if(in[385] <= 13.16) {
                if(in[430] <= 1.19) {
                    cls_hist[0]++;
                }
                else {
                    if(in[420] <= 0.52) {
                        cls_hist[0]++;
                    }
                    else {
                        cls_hist[0]++;
                    }
                }
            }
            else {
                cls_hist[1]++;
            }
        }
    }
    else {
        if(in[182] <= 1.35) {
            if(in[485] <= -0.27) {
                cls_hist[0]++;
            }
            else {
                cls_hist[2]++;
            }
        }
        else {
            cls_hist[0]++;
        }
    }
    
	// tree 7
    if(in[421] <= 12.77) {
        if(in[130] <= 1.26) {
            if(in[171] <= -0.94) {
                if(in[355] <= 10.69) {
                    cls_hist[0]++;
                }
                else {
                    cls_hist[1]++;
                }
            }
            else {
                if(in[409] <= 12.53) {
                    if(in[451] <= 12.32) {
                        cls_hist[0]++;
                    }
                    else {
                        cls_hist[1]++;
                    }
                }
                else {
                    if(in[492] <= 3.33) {
                        cls_hist[1]++;
                    }
                    else {
                        cls_hist[0]++;
                    }
                }
            }
        }
        else {
            if(in[528] <= 0.86) {
                if(in[180] <= -3.50) {
                    cls_hist[0]++;
                }
                else {
                    cls_hist[2]++;
                }
            }
            else {
                cls_hist[0]++;
            }
        }
    }
    else {
        cls_hist[1]++;
    }
    
	// tree 8
    if(in[331] <= 12.24) {
        if(in[403] <= 11.59) {
            if(in[166] <= 0.69) {
                if(in[445] <= 13.10) {
                    if(in[391] <= 11.84) {
                        cls_hist[0]++;
                    }
                    else {
                        cls_hist[1]++;
                    }
                }
                else {
                    cls_hist[1]++;
                }
            }
            else {
                if(in[280] <= -0.57) {
                    if(in[174] <= 0.47) {
                        cls_hist[2]++;
                    }
                    else {
                        cls_hist[0]++;
                    }
                }
                else {
                    if(in[187] <= 5.70) {
                        cls_hist[1]++;
                    }
                    else {
                        cls_hist[0]++;
                    }
                }
            }
        }
        else {
            if(in[330] <= 1.02) {
                cls_hist[1]++;
            }
            else {
                cls_hist[0]++;
            }
        }
    }
    else {
        cls_hist[1]++;
    }
    
	// tree 9
    if(in[133] <= 10.98) {
        if(in[94] <= 1.13) {
            if(in[264] <= -1.19) {
                if(in[118] <= 0.92) {
                    cls_hist[0]++;
                }
                else {
                    if(in[328] <= -0.74) {
                        cls_hist[2]++;
                    }
                    else {
                        cls_hist[0]++;
                    }
                }
            }
            else {
                if(in[343] <= 11.41) {
                    if(in[403] <= 12.84) {
                        cls_hist[0]++;
                    }
                    else {
                        cls_hist[1]++;
                    }
                }
                else {
                    if(in[338] <= 2.41) {
                        cls_hist[0]++;
                    }
                    else {
                        cls_hist[1]++;
                    }
                }
            }
        }
        else {
            if(in[529] <= 9.43) {
                cls_hist[0]++;
            }
            else {
                if(in[292] <= -0.25) {
                    cls_hist[2]++;
                }
                else {
                    cls_hist[0]++;
                }
            }
        }
    }
    else {
        if(in[247] <= 8.42) {
            if(in[64] <= 0.41) {
                cls_hist[1]++;
            }
            else {
                if(in[457] <= 9.86) {
                    cls_hist[2]++;
                }
                else {
                    cls_hist[0]++;
                }
            }
        }
        else {
            if(in[181] <= 8.06) {
                cls_hist[2]++;
            }
            else {
                cls_hist[0]++;
            }
        }
    }
    
	// tree 10
    if(in[322] <= -1.51) {
        if(in[170] <= 0.39) {
            cls_hist[2]++;
        }
        else {
            if(in[420] <= 0.45) {
                if(in[84] <= 0.36) {
                    if(in[457] <= 9.75) {
                        cls_hist[2]++;
                    }
                    else {
                        cls_hist[0]++;
                    }
                }
                else {
                    cls_hist[1]++;
                }
            }
            else {
                cls_hist[0]++;
            }
        }
    }
    else {
        if(in[373] <= 12.16) {
            if(in[151] <= 10.88) {
                if(in[340] <= -1.20) {
                    if(in[541] <= 9.60) {
                        cls_hist[0]++;
                    }
                    else {
                        cls_hist[2]++;
                    }
                }
                else {
                    if(in[160] <= 1.31) {
                        cls_hist[0]++;
                    }
                    else {
                        cls_hist[0]++;
                    }
                }
            }
            else {
                if(in[93] <= -0.09) {
                    if(in[208] <= -0.32) {
                        cls_hist[0]++;
                    }
                    else {
                        cls_hist[1]++;
                    }
                }
                else {
                    cls_hist[0]++;
                }
            }
        }
        else {
            if(in[587] <= -0.20) {
                cls_hist[0]++;
            }
            else {
                cls_hist[1]++;
            }
        }
    }
    
	// tree 11
    if(in[166] <= 0.86) {
        if(in[427] <= 11.61) {
            if(in[391] <= 11.91) {
                if(in[176] <= -1.12) {
                    if(in[94] <= 0.69) {
                        cls_hist[0]++;
                    }
                    else {
                        cls_hist[2]++;
                    }
                }
                else {
                    if(in[189] <= -0.96) {
                        cls_hist[0]++;
                    }
                    else {
                        cls_hist[0]++;
                    }
                }
            }
            else {
                if(in[129] <= -0.12) {
                    cls_hist[1]++;
                }
                else {
                    cls_hist[0]++;
                }
            }
        }
        else {
            cls_hist[1]++;
        }
    }
    else {
        if(in[346] <= -0.72) {
            if(in[254] <= 1.33) {
                if(in[4] <= 0.46) {
                    cls_hist[2]++;
                }
                else {
                    cls_hist[0]++;
                }
            }
            else {
                if(in[250] <= 0.12) {
                    cls_hist[1]++;
                }
                else {
                    cls_hist[0]++;
                }
            }
        }
        else {
            if(in[153] <= -0.94) {
                cls_hist[1]++;
            }
            else {
                cls_hist[0]++;
            }
        }
    }
    
	// tree 12
    if(in[310] <= -1.32) {
        if(in[168] <= -0.66) {
            cls_hist[2]++;
        }
        else {
            if(in[127] <= 11.94) {
                cls_hist[0]++;
            }
            else {
                cls_hist[1]++;
            }
        }
    }
    else {
        if(in[221] <= -0.54) {
            if(in[0] <= 0.91) {
                if(in[33] <= 0.03) {
                    if(in[368] <= 4.08) {
                        cls_hist[1]++;
                    }
                    else {
                        cls_hist[0]++;
                    }
                }
                else {
                    cls_hist[0]++;
                }
            }
            else {
                cls_hist[0]++;
            }
        }
        else {
            if(in[196] <= 2.13) {
                if(in[187] <= 12.52) {
                    if(in[385] <= 13.52) {
                        cls_hist[0]++;
                    }
                    else {
                        cls_hist[1]++;
                    }
                }
                else {
                    cls_hist[1]++;
                }
            }
            else {
                cls_hist[1]++;
            }
        }
    }
    
	// tree 13
    if(in[280] <= -0.98) {
        if(in[331] <= 9.61) {
            cls_hist[0]++;
        }
        else {
            if(in[397] <= 8.36) {
                cls_hist[1]++;
            }
            else {
                if(in[230] <= 0.33) {
                    cls_hist[2]++;
                }
                else {
                    cls_hist[0]++;
                }
            }
        }
    }
    else {
        if(in[157] <= 10.53) {
            if(in[148] <= 0.86) {
                if(in[415] <= 13.46) {
                    if(in[451] <= 12.90) {
                        cls_hist[0]++;
                    }
                    else {
                        cls_hist[1]++;
                    }
                }
                else {
                    cls_hist[1]++;
                }
            }
            else {
                if(in[222] <= -0.94) {
                    if(in[487] <= 9.32) {
                        cls_hist[0]++;
                    }
                    else {
                        cls_hist[2]++;
                    }
                }
                else {
                    cls_hist[0]++;
                }
            }
        }
        else {
            if(in[25] <= 9.62) {
                cls_hist[0]++;
            }
            else {
                if(in[151] <= 10.97) {
                    if(in[219] <= -0.38) {
                        cls_hist[1]++;
                    }
                    else {
                        cls_hist[0]++;
                    }
                }
                else {
                    cls_hist[1]++;
                }
            }
        }
    }
    
	// tree 14
    if(in[328] <= -1.48) {
        if(in[124] <= 0.88) {
            if(in[535] <= 9.64) {
                cls_hist[0]++;
            }
            else {
                cls_hist[1]++;
            }
        }
        else {
            cls_hist[2]++;
        }
    }
    else {
        if(in[252] <= -0.01) {
            if(in[130] <= 1.59) {
                if(in[403] <= 11.16) {
                    if(in[94] <= 1.04) {
                        cls_hist[0]++;
                    }
                    else {
                        cls_hist[2]++;
                    }
                }
                else {
                    cls_hist[1]++;
                }
            }
            else {
                if(in[391] <= 10.84) {
                    cls_hist[2]++;
                }
                else {
                    cls_hist[0]++;
                }
            }
        }
        else {
            if(in[379] <= 13.39) {
                if(in[397] <= 12.35) {
                    if(in[462] <= -0.54) {
                        cls_hist[0]++;
                    }
                    else {
                        cls_hist[0]++;
                    }
                }
                else {
                    cls_hist[1]++;
                }
            }
            else {
                cls_hist[1]++;
            }
        }
    }
    
	// tree 15
    if(in[328] <= -1.55) {
        if(in[272] <= 0.52) {
            cls_hist[2]++;
        }
        else {
            if(in[217] <= 4.78) {
                cls_hist[1]++;
            }
            else {
                cls_hist[0]++;
            }
        }
    }
    else {
        if(in[106] <= 1.28) {
            if(in[409] <= 12.96) {
                if(in[379] <= 17.19) {
                    if(in[177] <= -1.03) {
                        cls_hist[0]++;
                    }
                    else {
                        cls_hist[0]++;
                    }
                }
                else {
                    cls_hist[1]++;
                }
            }
            else {
                cls_hist[1]++;
            }
        }
        else {
            if(in[384] <= 1.09) {
                if(in[14] <= 2.06) {
                    cls_hist[2]++;
                }
                else {
                    cls_hist[0]++;
                }
            }
            else {
                if(in[254] <= 0.40) {
                    if(in[519] <= 0.02) {
                        cls_hist[2]++;
                    }
                    else {
                        cls_hist[0]++;
                    }
                }
                else {
                    cls_hist[0]++;
                }
            }
        }
    }
    
	// tree 16
    if(in[154] <= 0.97) {
        if(in[97] <= 10.65) {
            if(in[61] <= 11.22) {
                if(in[445] <= 14.14) {
                    if(in[175] <= 13.93) {
                        cls_hist[0]++;
                    }
                    else {
                        cls_hist[1]++;
                    }
                }
                else {
                    cls_hist[1]++;
                }
            }
            else {
                if(in[388] <= 0.64) {
                    cls_hist[0]++;
                }
                else {
                    cls_hist[1]++;
                }
            }
        }
        else {
            if(in[357] <= 0.12) {
                if(in[127] <= 11.34) {
                    cls_hist[0]++;
                }
                else {
                    if(in[307] <= 9.51) {
                        cls_hist[1]++;
                    }
                    else {
                        cls_hist[0]++;
                    }
                }
            }
            else {
                if(in[526] <= 0.09) {
                    cls_hist[1]++;
                }
                else {
                    cls_hist[0]++;
                }
            }
        }
    }
    else {
        if(in[280] <= -0.29) {
            if(in[258] <= -0.25) {
                if(in[177] <= -0.75) {
                    cls_hist[1]++;
                }
                else {
                    cls_hist[2]++;
                }
            }
            else {
                cls_hist[0]++;
            }
        }
        else {
            if(in[373] <= 9.87) {
                cls_hist[0]++;
            }
            else {
                if(in[22] <= 0.07) {
                    cls_hist[2]++;
                }
                else {
                    cls_hist[0]++;
                }
            }
        }
    }
    
	// tree 17
    if(in[121] <= 13.41) {
        if(in[391] <= 13.97) {
            if(in[328] <= -1.55) {
                if(in[88] <= 0.39) {
                    cls_hist[0]++;
                }
                else {
                    cls_hist[2]++;
                }
            }
            else {
                if(in[346] <= -1.15) {
                    if(in[100] <= 0.86) {
                        cls_hist[0]++;
                    }
                    else {
                        cls_hist[2]++;
                    }
                }
                else {
                    if(in[292] <= -2.27) {
                        cls_hist[2]++;
                    }
                    else {
                        cls_hist[0]++;
                    }
                }
            }
        }
        else {
            cls_hist[1]++;
        }
    }
    else {
        if(in[193] <= 6.92) {
            cls_hist[1]++;
        }
        else {
            cls_hist[0]++;
        }
    }
    
	// tree 18
    if(in[121] <= 10.87) {
        if(in[310] <= -1.28) {
            if(in[570] <= 0.68) {
                if(in[433] <= 8.43) {
                    cls_hist[0]++;
                }
                else {
                    cls_hist[2]++;
                }
            }
            else {
                cls_hist[0]++;
            }
        }
        else {
            if(in[79] <= 10.51) {
                if(in[169] <= 13.12) {
                    if(in[358] <= -1.52) {
                        cls_hist[0]++;
                    }
                    else {
                        cls_hist[0]++;
                    }
                }
                else {
                    if(in[197] <= -0.21) {
                        cls_hist[0]++;
                    }
                    else {
                        cls_hist[1]++;
                    }
                }
            }
            else {
                if(in[271] <= 8.25) {
                    if(in[433] <= 9.02) {
                        cls_hist[0]++;
                    }
                    else {
                        cls_hist[1]++;
                    }
                }
                else {
                    cls_hist[0]++;
                }
            }
        }
    }
    else {
        if(in[99] <= -0.01) {
            if(in[234] <= 0.67) {
                if(in[570] <= -0.28) {
                    if(in[395] <= 0.09) {
                        cls_hist[2]++;
                    }
                    else {
                        cls_hist[0]++;
                    }
                }
                else {
                    cls_hist[1]++;
                }
            }
            else {
                cls_hist[0]++;
            }
        }
        else {
            if(in[268] <= -1.38) {
                cls_hist[2]++;
            }
            else {
                if(in[211] <= 4.50) {
                    cls_hist[1]++;
                }
                else {
                    cls_hist[0]++;
                }
            }
        }
    }
    
	// tree 19
    if(in[103] <= 10.90) {
        if(in[148] <= 1.16) {
            if(in[136] <= 0.99) {
                if(in[181] <= 12.84) {
                    if(in[322] <= -0.66) {
                        cls_hist[0]++;
                    }
                    else {
                        cls_hist[0]++;
                    }
                }
                else {
                    cls_hist[1]++;
                }
            }
            else {
                if(in[310] <= -0.60) {
                    if(in[569] <= -0.06) {
                        cls_hist[0]++;
                    }
                    else {
                        cls_hist[2]++;
                    }
                }
                else {
                    cls_hist[0]++;
                }
            }
        }
        else {
            if(in[258] <= -0.55) {
                if(in[79] <= 10.34) {
                    cls_hist[2]++;
                }
                else {
                    cls_hist[0]++;
                }
            }
            else {
                cls_hist[0]++;
            }
        }
    }
    else {
        if(in[276] <= 0.83) {
            if(in[211] <= 7.88) {
                if(in[194] <= 0.96) {
                    cls_hist[0]++;
                }
                else {
                    cls_hist[1]++;
                }
            }
            else {
                if(in[558] <= 0.47) {
                    cls_hist[0]++;
                }
                else {
                    cls_hist[2]++;
                }
            }
        }
        else {
            cls_hist[0]++;
        }
    }
    
}
#endif
