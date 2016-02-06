#ifndef _CUST_BATTERY_METER_TABLE_H
#define _CUST_BATTERY_METER_TABLE_H

#include <mach/mt_typedefs.h>

// ============================================================
// define
// ============================================================
#define BAT_NTC_10 1
#define BAT_NTC_47 0

#if (BAT_NTC_10 == 1)
#define RBAT_PULL_UP_R             16900	
#if defined(__M8512__)
#define RBAT_PULL_DOWN_R		   33000	
#else
#define RBAT_PULL_DOWN_R		   27000	
#endif
#endif

#if (BAT_NTC_47 == 1)
#define RBAT_PULL_UP_R             61900	
#define RBAT_PULL_DOWN_R		  100000	
#endif
#define RBAT_PULL_UP_VOLT          1800



// ============================================================
// ENUM
// ============================================================

// ============================================================
// structure
// ============================================================

// ============================================================
// typedef
// ============================================================
typedef struct _BATTERY_PROFILE_STRUC
{
    kal_int32 percentage;
    kal_int32 voltage;
} BATTERY_PROFILE_STRUC, *BATTERY_PROFILE_STRUC_P;

typedef struct _R_PROFILE_STRUC
{
    kal_int32 resistance; // Ohm
    kal_int32 voltage;
} R_PROFILE_STRUC, *R_PROFILE_STRUC_P;

typedef enum
{
    T1_0C,
    T2_25C,
    T3_50C
} PROFILE_TEMPERATURE;

// ============================================================
// External Variables
// ============================================================

// ============================================================
// External function
// ============================================================

// ============================================================
// <DOD, Battery_Voltage> Table
// ============================================================
#if (BAT_NTC_10 == 1)
    BATT_TEMPERATURE Batt_Temperature_Table[] = {
        {-20,68237},
        {-15,53650},
        {-10,42506},
        { -5,33892},
        {  0,27219},
        {  5,22021},
        { 10,17926},
        { 15,14674},
        { 20,12081},
        { 25,10000},
        { 30,8315},
        { 35,6948},
        { 40,5834},
        { 45,4917},
        { 50,4161},
        { 55,3535},
        { 60,3014}
    };
#endif

#if (BAT_NTC_47 == 1)
    BATT_TEMPERATURE Batt_Temperature_Table[] = {
        {-20,483954},
        {-15,360850},
        {-10,271697},
        { -5,206463},
        {  0,158214},
        {  5,122259},
        { 10,95227},
        { 15,74730},
        { 20,59065},
        { 25,47000},
        { 30,37643},
        { 35,30334},
        { 40,24591},
        { 45,20048},
        { 50,16433},
        { 55,13539},
        { 60,11210}        
    };
#endif

#if defined(__M8510__)||defined(__M8512__)|| defined(__M8513__)
// T0 -10C
BATTERY_PROFILE_STRUC battery_profile_t0[] =
{
    {0  ,4177},
    {2  ,4162},
    {4  ,4148},
    {6  ,4124},
    {8  ,4084},
    {10 ,4053},
    {12 ,4025},
    {14 ,4001},
    {16 ,3986},
    {18 ,3975},
    {19 ,3965},
    {21 ,3955},
    {23 ,3944},
    {25 ,3932},
    {27 ,3920},
    {29 ,3907},
    {31 ,3894},
    {33 ,3884},
    {35 ,3874},
    {37 ,3865},
    {39 ,3857},
    {41 ,3849},
    {43 ,3842},
    {45 ,3836},
    {47 ,3829},
    {49 ,3824},
    {51 ,3819},
    {52 ,3813},
    {54 ,3807},
    {56 ,3802},
    {58 ,3797},
    {60 ,3791},
    {62 ,3787},
    {64 ,3782},
    {66 ,3776},
    {68 ,3772},
    {70 ,3768},
    {72 ,3762},
    {74 ,3756},
    {76 ,3751},
    {78 ,3744},
    {80 ,3736},
    {82 ,3729},
    {84 ,3721},
    {85 ,3714},
    {87 ,3708},
    {89 ,3703},
    {91 ,3692},
    {93 ,3672},
    {95 ,3634},
    {97 ,3570},
    {99 ,3499},
    {99 ,3472},
    {99 ,3458},
    {100,3450},
    {100,3444},
    {100,3439},
    {100,3436},
    {100,3433},
    {100,3430},
    {100,3428},
    {100,3427},
    {100,3426},
    {100,3424},
    {100,3423},
    {100,3422},
    {100,3421},
    {100,3419},
    {100,3418},
    {100,3417},
    {100,3415},
    {100,3414},
    {100,3413},
    {100,3412},
    {100,3400},
};      
        
// T1 0C 
BATTERY_PROFILE_STRUC battery_profile_t1[] =
{
    {0  ,4172},
    {2  ,4144},
    {3  ,4117},
    {5  ,4094},
    {6  ,4078},
    {8  ,4061},
    {9  ,4041},
    {11 ,4023},
    {13 ,4006},
    {14 ,3995},
    {16 ,3986},
    {17 ,3978},
    {19 ,3970},
    {20 ,3961},
    {22 ,3951},
    {24 ,3943},
    {25 ,3934},
    {27 ,3925},
    {28 ,3916},
    {30 ,3906},
    {31 ,3895},
    {33 ,3884},
    {35 ,3872},
    {36 ,3861},
    {38 ,3852},
    {39 ,3842},
    {41 ,3836},
    {43 ,3829},
    {44 ,3824},
    {46 ,3819},
    {47 ,3813},
    {49 ,3807},
    {50 ,3804},
    {52 ,3800},
    {54 ,3797},
    {55 ,3794},
    {57 ,3791},
    {58 ,3787},
    {60 ,3785},
    {61 ,3783},
    {63 ,3780},
    {65 ,3778},
    {66 ,3775},
    {68 ,3774},
    {69 ,3771},
    {71 ,3768},
    {72 ,3765},
    {74 ,3761},
    {76 ,3757},
    {77 ,3753},
    {79 ,3748},
    {80 ,3742},
    {82 ,3736},
    {83 ,3730},
    {85 ,3722},
    {87 ,3714},
    {88 ,3705},
    {90 ,3700},
    {91 ,3697},
    {93 ,3691},
    {94 ,3676},
    {96 ,3636},
    {98 ,3571},
    {99 ,3491},
    {99 ,3455},
    {100,3431},
    {100,3413},
    {100,3400},
    {100,3391},
    {100,3383},
    {100,3377},
    {100,3371},
    {100,3367},
    {100,3364},
    {100,3364},
};           

// T2 25C
BATTERY_PROFILE_STRUC battery_profile_t2[] =
{
    {0  ,4183},
    {1  ,4164},
    {3  ,4148},
    {4  ,4133},
    {6  ,4118},
    {7  ,4104},
    {9  ,4091},
    {10 ,4078},
    {12 ,4066},
    {13 ,4053},
    {15 ,4040},
    {16 ,4026},
    {18 ,4014},
    {19 ,4003},
    {21 ,3993},
    {22 ,3983},
    {23 ,3973},
    {25 ,3964},
    {26 ,3955},
    {28 ,3945},
    {29 ,3936},
    {31 ,3928},
    {32 ,3919},
    {34 ,3911},
    {35 ,3901},
    {37 ,3892},
    {38 ,3881},
    {40 ,3867},
    {41 ,3856},
    {42 ,3846},
    {44 ,3837},
    {45 ,3831},
    {47 ,3824},
    {48 ,3819},
    {50 ,3813},
    {51 ,3809},
    {53 ,3804},
    {54 ,3800},
    {56 ,3796},
    {57 ,3793},
    {59 ,3790},
    {60 ,3787},
    {62 ,3784},
    {63 ,3781},
    {64 ,3779},
    {66 ,3777},
    {67 ,3775},
    {69 ,3773},
    {70 ,3769},
    {72 ,3763},
    {73 ,3759},
    {75 ,3753},
    {76 ,3748},
    {78 ,3744},
    {79 ,3739},
    {81 ,3735},
    {82 ,3729},
    {83 ,3723},
    {85 ,3715},
    {86 ,3710},
    {88 ,3701},
    {89 ,3691},
    {91 ,3687},
    {92 ,3685},
    {94 ,3680},
    {95 ,3665},
    {97 ,3620},
    {98 ,3551},
    {100,3446},
    {100,3288},
    {100,3267},
    {100,3261},
    {100,3257},
    {100,3254},
    {100,3254},
};     

// T3 50C
BATTERY_PROFILE_STRUC battery_profile_t3[] =
{
    {0  ,4189},
    {1  ,4170},
    {3  ,4155},
    {4  ,4139},
    {6  ,4124},
    {7  ,4110},
    {9  ,4097},
    {10 ,4083},
    {12 ,4071},
    {13 ,4058},
    {15 ,4046},
    {16 ,4033},
    {17 ,4021},
    {19 ,4010},
    {20 ,3999},
    {22 ,3988},
    {23 ,3978},
    {25 ,3967},
    {26 ,3958},
    {28 ,3948},
    {29 ,3939},
    {31 ,3930},
    {32 ,3922},
    {33 ,3913},
    {35 ,3905},
    {36 ,3896},
    {38 ,3887},
    {39 ,3875},
    {41 ,3861},
    {42 ,3848},
    {44 ,3839},
    {45 ,3832},
    {46 ,3825},
    {48 ,3820},
    {49 ,3813},
    {51 ,3809},
    {52 ,3804},
    {54 ,3800},
    {55 ,3795},
    {57 ,3792},
    {58 ,3788},
    {60 ,3784},
    {61 ,3782},
    {62 ,3779},
    {64 ,3777},
    {65 ,3774},
    {67 ,3771},
    {68 ,3769},
    {70 ,3763},
    {71 ,3754},
    {73 ,3746},
    {74 ,3741},
    {76 ,3736},
    {77 ,3730},
    {78 ,3726},
    {80 ,3722},
    {81 ,3717},
    {83 ,3712},
    {84 ,3704},
    {86 ,3697},
    {87 ,3690},
    {89 ,3681},
    {90 ,3673},
    {91 ,3672},
    {93 ,3668},
    {94 ,3660},
    {96 ,3631},
    {97 ,3577},
    {99 ,3500},
    {100,3381},
    {100,3268},
    {100,3254},
    {100,3249},
    {100,3247},
    {100,3247},
};           
#else
// T0 -10C
BATTERY_PROFILE_STRUC battery_profile_t0[] =
{
	{0   , 4161},         
	{2   , 4139},         
	{5   , 4113},         
	{7   , 4080},         
	{10  , 4037},         
	{12  , 4005},         
	{15  , 3986},         
	{17  , 3971},         
	{19  , 3956},         
	{22  , 3943},         
	{24  , 3928},         
	{27  , 3915},         
	{29  , 3898},         
	{32  , 3884},         
	{34  , 3869},         
	{36  , 3857},         
	{39  , 3846},         
	{41  , 3835},         
	{44  , 3826},         
	{46  , 3819},         
	{48  , 3813},         
	{51  , 3806},         
	{53  , 3800},         
	{56  , 3795},         
	{58  , 3792},         
	{61  , 3788},         
	{63  , 3783},         
	{65  , 3782},         
	{68  , 3778},         
	{70  , 3772},         
	{73  , 3768},         
	{75  , 3761},         
	{78  , 3752},         
	{80  , 3740},         
	{82  , 3729},         
	{85  , 3717},         
	{87  , 3708},         
	{90  , 3701},         
	{91  , 3695},         
	{93  , 3690},         
	{94  , 3687},         
	{95  , 3683},         
	{95  , 3681},         
	{96  , 3677},         
	{96  , 3672},         
	{97  , 3668},         
	{97  , 3661},         
	{97  , 3658},         
	{98  , 3653},         
	{98  , 3649},         
	{98  , 3642},         
	{98  , 3638},         
	{99  , 3633},         
	{99  , 3629},         
	{99  , 3625},         
	{99  , 3621},         
	{99  , 3619},         
	{99  , 3614},         
	{99  , 3612},         
	{99  , 3611},         
	{99  , 3608},          
  {100 , 3606},
  {100 , 3605},
  {100 , 3603},
  {100 , 3601},
  {100 , 3599},
  {100 , 3598},
  {100 , 3595},
  {100 , 3593},
  {100 , 3592},
	{100 , 3590}, 
	{100 , 3588},
	{100 , 3586},
	{100 , 3586},
	{100 , 3585},
	{100 , 3584}, 
	{100 , 3582}	       
};      
        
// T1 0C 
BATTERY_PROFILE_STRUC battery_profile_t1[] =
{
	{0   , 4160},         
	{2   , 4109},         
	{4   , 4080},         
	{7   , 4060},         
	{9   , 4041},         
	{11  , 4022},         
	{13  , 4006},         
	{15  , 3992},         
	{17  , 3978},         
	{20  , 3965},         
	{22  , 3954},         
	{24  , 3940},         
	{26  , 3928},         
	{28  , 3915},         
	{31  , 3900},         
	{33  , 3884},         
	{35  , 3868},         
	{37  , 3855},         
	{39  , 3842},         
	{41  , 3833},         
	{44  , 3824},         
	{46  , 3816},         
	{48  , 3811},         
	{50  , 3804},         
	{52  , 3800},         
	{54  , 3794},         
	{57  , 3789},         
	{59  , 3787},         
	{61  , 3785},         
	{63  , 3781},         
	{65  , 3779},         
	{67  , 3778},         
	{70  , 3774},         
	{72  , 3771},         
	{74  , 3765},         
	{76  , 3759},         
	{78  , 3748},         
	{81  , 3739},         
	{83  , 3727},         
	{85  , 3712},         
	{87  , 3701},         
	{89  , 3694},         
	{91  , 3685},         
	{94  , 3677},         
	{96  , 3654},         
	{97  , 3628},         
	{97  , 3607},         
	{98  , 3588},         
	{98  , 3572},         
	{99  , 3560},         
	{99  , 3550},         
	{99  , 3540},         
	{99  , 3532},         
	{99  , 3527},         
	{99  , 3520},         
	{99  , 3513},         
	{99  , 3509},         
	{99  , 3505},         
	{99  , 3501},         
	{100 , 3498},         
	{100 , 3495},          
  {100 , 3491},
  {100 , 3488},
  {100 , 3487},
  {100 , 3483},
  {100 , 3482},
  {100 , 3479},
  {100 , 3479},
  {100 , 3476},
  {100 , 3475},
	{100 , 3473}, 
	{100 , 3472},
	{100 , 3468},
	{100 , 3467},
	{100 , 3466},
	{100 , 3463}, 
	{100 , 3463}	       
};           

// T2 25C
BATTERY_PROFILE_STRUC battery_profile_t2[] =
{
	{0   , 4179},         
	{2   , 4155},         
	{4   , 4136},         
	{6   , 4118},         
	{8   , 4098},         
	{10  , 4082},         
	{12  , 4065},         
	{14  , 4048},         
	{16  , 4029},         
	{18  , 4014},         
	{20  , 4000},         
	{22  , 3986},         
	{24  , 3973},         
	{26  , 3959},         
	{28  , 3946},         
	{30  , 3936},         
	{32  , 3924},         
	{34  , 3914},         
	{36  , 3901},         
	{38  , 3889},         
	{40  , 3867},         
	{42  , 3850},         
	{44  , 3837},         
	{46  , 3827},         
	{48  , 3819},         
	{50  , 3812},         
	{52  , 3806},         
	{54  , 3800},         
	{56  , 3796},         
	{58  , 3791},         
	{60  , 3788},         
	{62  , 3781},         
	{64  , 3778},         
	{66  , 3777},         
	{68  , 3774},         
	{70  , 3771},         
	{72  , 3769},         
	{74  , 3762},         
	{76  , 3755},         
	{78  , 3746},         
	{80  , 3739},         
	{82  , 3730},         
	{84  , 3718},         
	{86  , 3704},         
	{88  , 3688},         
	{90  , 3681},         
	{92  , 3677},         
	{94  , 3670},         
	{96  , 3644},         
	{98  , 3562},         
	{100 , 3408},         
	{100 , 3349},         
	{100 , 3329},         
	{100 , 3319},         
	{100 , 3313},         
	{100 , 3309},         
	{100 , 3304},         
	{100 , 3302},         
	{100 , 3300},         
	{100 , 3296},         
	{100 , 3295},          
  {100 , 3297},
  {100 , 3296},
  {100 , 3296},
  {100 , 3296},
  {100 , 3293},
  {100 , 3289},
  {100 , 3289},
  {100 , 3286},
  {100 , 3285},
	{100 , 3284}, 
	{100 , 3284},
	{100 , 3284},
	{100 , 3284},
	{100 , 3283},
	{100 , 3282}, 
	{100 , 3281}	       
};     

// T3 50C
BATTERY_PROFILE_STRUC battery_profile_t3[] =
{
	{0   , 4186},         
	{2   , 4163},         
	{4   , 4143},         
	{6   , 4122},         
	{9   , 4104},         
	{11  , 4083},         
	{13  , 4065},         
	{15  , 4048},         
	{17  , 4030},         
	{19  , 4013},         
	{21  , 3997},         
	{24  , 3982},         
	{26  , 3968},         
	{28  , 3953},         
	{30  , 3941},         
	{32  , 3928},         
	{34  , 3914},         
	{36  , 3897},         
	{39  , 3877},         
	{41  , 3862},         
	{43  , 3848},         
	{45  , 3838},         
	{47  , 3828},         
	{49  , 3820},         
	{51  , 3814},         
	{54  , 3805},         
	{56  , 3799},         
	{58  , 3793},         
	{60  , 3788},         
	{62  , 3783},         
	{64  , 3776},         
	{66  , 3771},         
	{69  , 3762},         
	{71  , 3752},         
	{73  , 3745},         
	{75  , 3737},         
	{77  , 3729},         
	{79  , 3721},         
	{81  , 3710},         
	{84  , 3698},         
	{86  , 3684},         
	{88  , 3672},         
	{90  , 3669},         
	{92  , 3665},         
	{94  , 3654},         
	{96  , 3604},         
	{99  , 3508},         
	{100 , 3356},         
	{100 , 3331},         
	{100 , 3318},         
	{100 , 3313},         
	{100 , 3308},         
	{100 , 3306},         
	{100 , 3300},         
	{100 , 3298},         
	{100 , 3293},         
	{100 , 3292},         
	{100 , 3291},         
	{100 , 3287},         
	{100 , 3284},         
	{100 , 3283},          
  {100 , 3280},
  {100 , 3279},
  {100 , 3278},
  {100 , 3276},
  {100 , 3276},
  {100 , 3275},
  {100 , 3273},
  {100 , 3272},
  {100 , 3272},
	{100 , 3273}, 
	{100 , 3274},
	{100 , 3272},
	{100 , 3270},
	{100 , 3271},
	{100 , 3270}, 
	{100 , 3269}	       
};           
#endif
// battery profile for actual temperature. The size should be the same as T1, T2 and T3
BATTERY_PROFILE_STRUC battery_profile_temperature[] =
{
  {0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },  
	{0  , 0 }, 
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },  
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 }
};    

// ============================================================
// <Rbat, Battery_Voltage> Table
// ============================================================
// T0 -10C
R_PROFILE_STRUC r_profile_t0[] =
{
	{360  , 4161},         
	{498  , 4139},         
	{523  , 4113},         
	{553  , 4080},         
	{623  , 4037},         
	{785  , 4005},         
	{818  , 3986},         
	{835  , 3971},         
	{838  , 3956},         
	{845  , 3943},         
	{845  , 3928},         
	{848  , 3915},         
	{845  , 3898},         
	{838  , 3884},         
	{838  , 3869},         
	{840  , 3857},         
	{843  , 3846},         
	{843  , 3835},         
	{850  , 3826},         
	{863  , 3819},         
	{873  , 3813},         
	{883  , 3806},         
	{893  , 3800},         
	{905  , 3795},         
	{915  , 3792},         
	{928  , 3788},         
	{930  , 3783},         
	{948  , 3782},         
	{960  , 3778},         
	{968  , 3772},         
	{985  , 3768},         
	{998  , 3761},         
	{1013 , 3752},         
	{1025 , 3740},         
	{1050 , 3729},         
	{1080 , 3717},         
	{1128 , 3708},         
	{1208 , 3701},         
	{1240 , 3695},         
	{1228 , 3690},         
	{1220 , 3687},         
	{1213 , 3683},         
	{1205 , 3681},         
	{1195 , 3677},         
	{1185 , 3672},         
	{1173 , 3668},         
	{1160 , 3661},         
	{1150 , 3658},         
	{1135 , 3653},         
	{1125 , 3649},         
	{1113 , 3642},         
	{1103 , 3638},         
	{1085 , 3633},         
	{1080 , 3629},         
	{1073 , 3625},         
	{1060 , 3621},         
	{1055 , 3619},         
	{1045 , 3614},         
	{1043 , 3612},         
	{1030 , 3611},         
	{1030 , 3608},          
  {1028 , 3606},
  {1013 , 3605},
  {1015 , 3603},
  {1008 , 3601},
  {1003 , 3599},
  {1008 , 3598},
  {998  , 3595},
  {1000 , 3593},
  {1003 , 3592},
	{1005 , 3590}, 
	{980  , 3588},
	{980  , 3586},
	{993  , 3586},
	{983  , 3585},
	{983  , 3584}, 
	{983  , 3582}	       
};      

// T1 0C
R_PROFILE_STRUC r_profile_t1[] =
{
	{268  , 4160},         
	{388  , 4109},         
	{468  , 4080},         
	{483  , 4060},         
	{495  , 4041},         
	{498  , 4022},         
	{513  , 4006},         
	{518  , 3992},         
	{523  , 3978},         
	{523  , 3965},         
	{535  , 3954},         
	{535  , 3940},         
	{533  , 3928},         
	{533  , 3915},         
	{525  , 3900},         
	{518  , 3884},         
	{508  , 3868},         
	{500  , 3855},         
	{495  , 3842},         
	{495  , 3833},         
	{495  , 3824},         
	{493  , 3816},         
	{500  , 3811},         
	{508  , 3804},         
	{515  , 3800},         
	{518  , 3794},         
	{523  , 3789},         
	{533  , 3787},         
	{543  , 3785},         
	{543  , 3781},         
	{545  , 3779},         
	{555  , 3778},         
	{555  , 3774},         
	{565  , 3771},         
	{570  , 3765},         
	{580  , 3759},         
	{580  , 3748},         
	{595  , 3739},         
	{603  , 3727},         
	{608  , 3712},         
	{630  , 3701},         
	{673  , 3694},         
	{750  , 3685},         
	{890  , 3677},         
	{1123 , 3654},         
	{1073 , 3628},         
	{1020 , 3607},         
	{978  , 3588},         
	{935  , 3572},         
	{908  , 3560},         
	{885  , 3550},         
	{850  , 3540},         
	{833  , 3532},         
	{828  , 3527},         
	{808  , 3520},         
	{790  , 3513},         
	{785  , 3509},         
	{768  , 3505},         
	{755  , 3501},         
	{763  , 3498},         
	{750  , 3495},          
  {748  , 3491},
  {725  , 3488},
  {728  , 3487},
  {730  , 3483},
  {705  , 3482},
  {705  , 3479},
  {725  , 3479},
  {713  , 3476},
  {715  , 3475},
	{715  , 3473}, 
	{718  , 3472},
	{718  , 3468},
	{718  , 3467},
	{695  , 3466},
	{668  , 3463}, 
	{690  , 3463}	       
};     

// T2 25C
R_PROFILE_STRUC r_profile_t2[] =
{
	{150  , 4179},         
	{205  , 4155},         
	{210  , 4136},         
	{215  , 4118},         
	{215  , 4098},         
	{223  , 4082},         
	{228  , 4065},         
	{230  , 4048},         
	{230  , 4029},         
	{238  , 4014},         
	{240  , 4000},         
	{245  , 3986},         
	{250  , 3973},         
	{250  , 3959},         
	{253  , 3946},         
	{265  , 3936},         
	{268  , 3924},         
	{270  , 3914},         
	{270  , 3901},         
	{268  , 3889},         
	{240  , 3867},         
	{223  , 3850},         
	{218  , 3837},         
	{213  , 3827},         
	{213  , 3819},         
	{215  , 3812},         
	{220  , 3806},         
	{223  , 3800},         
	{225  , 3796},         
	{228  , 3791},         
	{233  , 3788},         
	{223  , 3781},         
	{230  , 3778},         
	{233  , 3777},         
	{238  , 3774},         
	{238  , 3771},         
	{238  , 3769},         
	{230  , 3762},         
	{228  , 3755},         
	{220  , 3746},         
	{225  , 3739},         
	{230  , 3730},         
	{230  , 3718},         
	{233  , 3704},         
	{223  , 3688},         
	{228  , 3681},         
	{243  , 3677},         
	{263  , 3670},         
	{283  , 3644},         
	{315  , 3562},         
	{403  , 3408},         
	{380  , 3349},         
	{330  , 3329},         
	{305  , 3319},         
	{288  , 3313},         
	{275  , 3309},         
	{273  , 3304},         
	{260  , 3302},         
	{260  , 3300},         
	{248  , 3296},         
	{243  , 3295},          
  {250  , 3297},
  {248  , 3296},
  {243  , 3296},
  {243  , 3296},
  {235  , 3293},
  {223  , 3289},
  {233  , 3289},
  {228  , 3286},
  {223  , 3285},
	{225  , 3284}, 
	{225  , 3284},
	{218  , 3284},
	{220  , 3284},
	{218  , 3283},
	{220  , 3282}, 
	{213  , 3281}	       
}; 

// T3 50C
R_PROFILE_STRUC r_profile_t3[] =
{
	{150  , 4186},         
	{175  , 4163},         
	{183  , 4143},         
	{185  , 4122},         
	{195  , 4104},         
	{193  , 4083},         
	{198  , 4065},         
	{200  , 4048},         
	{200  , 4030},         
	{205  , 4013},         
	{210  , 3997},         
	{215  , 3982},         
	{215  , 3968},         
	{218  , 3953},         
	{223  , 3941},         
	{233  , 3928},         
	{235  , 3914},         
	{225  , 3897},         
	{208  , 3877},         
	{200  , 3862},         
	{198  , 3848},         
	{195  , 3838},         
	{195  , 3828},         
	{190  , 3820},         
	{200  , 3814},         
	{198  , 3805},         
	{203  , 3799},         
	{203  , 3793},         
	{205  , 3788},         
	{208  , 3783},         
	{203  , 3776},         
	{205  , 3771},         
	{200  , 3762},         
	{195  , 3752},         
	{195  , 3745},         
	{195  , 3737},         
	{195  , 3729},         
	{195  , 3721},         
	{198  , 3710},         
	{200  , 3698},         
	{200  , 3684},         
	{193  , 3672},         
	{203  , 3669},         
	{213  , 3665},         
	{230  , 3654},         
	{253  , 3604},         
	{283  , 3508},         
	{393  , 3356},         
	{333  , 3331},         
	{300  , 3318},         
	{288  , 3313},         
	{273  , 3308},         
	{268  , 3306},         
	{253  , 3300},         
	{250  , 3298},         
	{240  , 3293},         
	{235  , 3292},         
	{233  , 3291},         
	{225  , 3287},         
	{210  , 3284},         
	{210  , 3283},          
  {205  , 3280},
  {198  , 3279},
  {198  , 3278},
  {190  , 3276},
  {193  , 3276},
  {195  , 3275},
  {188  , 3273},
  {188  , 3272},
  {188  , 3272},
	{188  , 3273}, 
	{185  , 3274},
	{183  , 3272},
	{180  , 3270},
	{180  , 3271},
	{178  , 3270}, 
	{180  , 3269}	       
}; 

// r-table profile for actual temperature. The size should be the same as T1, T2 and T3
R_PROFILE_STRUC r_profile_temperature[] =
{
  {0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },  
	{0  , 0 }, 
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },  
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 }
};    

// ============================================================
// function prototype
// ============================================================
int fgauge_get_saddles(void);
BATTERY_PROFILE_STRUC_P fgauge_get_profile(kal_uint32 temperature);

int fgauge_get_saddles_r_table(void);
R_PROFILE_STRUC_P fgauge_get_profile_r_table(kal_uint32 temperature);

#endif	//#ifndef _CUST_BATTERY_METER_TABLE_H

