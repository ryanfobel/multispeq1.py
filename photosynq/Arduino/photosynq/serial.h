
// need anything more?   use Serial_Printf() or a cast

// declare serial functions

void Serial_Set(int s);
void Serial_Resend(void);
void Serial_Start_Recording(void);
void Serial_Stop_Recording(void);
void Serial_Printf(const char * format, ... );
int Serial_Read(void);
unsigned Serial_Available(void);
int Serial_Peek(void);
void Serial_Begin(int i = 57600);
void Serial_Flush_Input(void);
void Serial_Flush_Output(void);

#define Serial_Input_Float(terminators,timeout)  ((float)Serial_Input_Double(terminators,timeout))

char *Serial_Input_Chars(char *string, const char *terminators = (char *)"\r\n+", long unsigned int timeout = 0, unsigned int max_length = 0);
double Serial_Input_Double(const char *terminators = (const char *)"\r\n+", long unsigned int timeout = 0);
long Serial_Input_Long(const char *terminators = (const char *)"\r\n+", long unsigned int timeout = 0);
String Serial_Input_String(const char *terminators = (char *)"\r\n+", long unsigned int timeout = 0);

void Serial_Printf(const char *format, ... );

void Serial_Print (const char *str);
void Serial_Print (const String str);
void Serial_Print (const int i);
void Serial_Print (const unsigned u);
void Serial_Print (const float x, int places = 0);
void Serial_Print (const double xx, int places = 0);


void Serial_Print_Line (const char *str);
void Serial_Print_Line (const String str);
void Serial_Print_Line (const int i);
void Serial_Print_Line (const long i);
void Serial_Print_Line (const float x, int places = 0);

void Serial_Print_CRC (void);

// useful for error checking
#define assert(condition) \
 if (!(condition)) { \
    Serial_Printf("\"error\":\"in %s, line %d\"", __FILE__, __LINE__); \
 }

// could be useful with assert()
// get current address
// void* p = __builtin_return_address(0);
// printf("0x%x\n", (unsigned int)p);
