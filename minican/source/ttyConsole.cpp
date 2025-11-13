#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <variant>
#include <charconv>
#include <cctype>
#include <string_view>
#include <optional>

#include "ttyConsole.hpp"
#include "ch.h"
#include "hal.h"
#include "microrl/microrlShell.h"
#include "stdutil.h"
#include "printf.h"
#include "etl/string.h"
#include "UAVCAN/persistantParam.hpp"
#include "UAVCAN/persistantStorage.hpp"
#include "deviceRessource.hpp"

using FixedString = etl::string<128>;
using Value = std::variant<struct uavcan_protocol_param_Empty, int64_t, float, bool, FixedString>;

/*
  commande storage : st
  st -> liste tous les storages
  st clef -> retourne la valeur en ram
  st clef valeur -> set la valeur en ram puis en eeprom


 */
#ifdef TRACE

#if CONSOLE_DEV_USB
#include "usb_serial.h"
#endif

#ifdef CONSOLE_DEV_SD

/*===========================================================================*/
/* START OF EDITABLE SECTION                                           */
/*===========================================================================*/

// declaration des prototypes de fonction
// ces declarations sont necessaires pour remplir le tableau commands[] ci-dessous
using cmd_func_t =  void  (BaseSequentialStream *lchp, int argc,const char * const argv[]);
static cmd_func_t cmd_mem, cmd_uid, cmd_restart, cmd_param, cmd_uavparam, cmd_storage, cmd_can;
#if CH_DBG_STATISTICS
static cmd_func_t cmd_threads;
#endif
namespace {
  Value parse_value(etl::string_view input);
  std::optional<int> parse_value_int(etl::string_view input);
  FixedString &  appendFixed(FixedString &str, const uavcan_protocol_param_NumericValue &val);
  FixedString &  appendFixed(FixedString &str, const uavcan_protocol_param_Value &val);
  FixedString &  appendFixed(FixedString &str, const char *val);
  FixedString &  appendFixed(FixedString &str, const uavcan_protocol_param_GetSetResponse &val);
  FixedString &  appendFixed(FixedString &str, const uint8_t *ptr, size_t len);

  template <typename U>
  requires requires(FixedString& lhs, const U& rhs) {
    appendFixed(lhs, rhs);
  }
  FixedString& operator<<(FixedString& lhs, const U& rhs) {
    return appendFixed(lhs, rhs);
  }
  
  void  cmd_storage_list(); 
  void  cmd_storage_display(const char* key);
  void  cmd_storage_set(const char* key, const char* value);
  void  cmd_uavcan_storage_display(etl::string_view key);
  void  cmd_uavcan_storage_set(etl::string_view key, etl::string_view value);
}



static const ShellCommand commands[] = {
  {"mem", cmd_mem},		// affiche la mémoire libre/occupée
#if  CH_DBG_STATISTICS
  {"threads", cmd_threads},	// affiche pour chaque thread le taux d'utilisation de la pile et du CPU
#endif
  {"uid", cmd_uid},		// affiche le numéro d'identification unique du MCU
  {"param", cmd_param},		// fonction à but pedagogique qui affiche les
				//   paramètres qui lui sont passés

  {"st", cmd_storage},		// manage eeprom storage
  {"uavp", cmd_uavparam},	// manage parameters via UAVCan types
  {"can", cmd_can},		// print can speed, hardware/software version
  {"restart", cmd_restart},	// reboot MCU
 {NULL, NULL}			// marqueur de fin de tableau
};

/*
  definition de la fonction cmd_param asociée à la commande param (cf. commands[])
  cette fonction a but pédagogique affiche juste les paramètres fournis, et tente
  de convertir les paramètres en entier et en flottant, et affiche le resultat de
  cette conversion. 
  une fois le programme chargé dans la carte, essayer de rentrer 
  param toto 10 10.5 0x10
  dans le terminal d'eclipse pour voir le résultat 
 */
static void cmd_param(BaseSequentialStream *lchp, int argc,const char* const argv[])
{
  if (argc == 0) {  // si aucun paramètre n'a été passé à la commande param 
    chprintf(lchp, "pas de paramètre en entrée\r\n");
  } else { // sinon (un ou plusieurs pararamètres passés à la commande param 
    for (int argn=0; argn<argc; argn++) { // pour tous les paramètres
      chprintf(lchp, "le parametre %d/%d est %s\r\n", argn, argc-1, argv[argn]); // afficher

      // tentative pour voir si la chaine peut être convertie en nombre entier et en nombre flottant
      int entier = atoi (argv[argn]); // atoi converti si c'est possible une chaine en entier
      float flottant = atof (argv[argn]); // atof converti si c'est possible une chaine en flottant

      chprintf(lchp, "atoi(%s) = %d ;; atof(%s) = %.3f\r\n",
		argv[argn], entier, argv[argn], flottant);
    }
  }
}

#include <array>
#include <string_view>

#ifndef CAN_BITRATE
#define CAN_BITRATE -500   // exemple
#endif

// Nombre de chiffres décimaux d'un entier positif
consteval std::size_t digit_count(unsigned int v) {
    std::size_t n = 1;
    while (v >= 10) {
        v /= 10;
        ++n;
    }
    return n;
}

template<int BR>
consteval auto make_bitrate_array() {
    constexpr int abs_br = BR < 0 ? -BR : BR;
    static_assert(abs_br > 0, "CAN_BITRATE must be non-zero");

    constexpr bool mbits = (abs_br >= 1000) && (abs_br % 1000 == 0);
    constexpr unsigned value = mbits ? abs_br / 1000U : abs_br;

    constexpr std::string_view prefix{"can bitrate = "};
    constexpr std::string_view suffix{mbits ? " mbits/s" : " kbits/s"};

    constexpr std::size_t digits = digit_count(value);

    // +1 pour le '\0'
    constexpr std::size_t total = prefix.size() + digits + suffix.size() + 1;

    std::array<char, total> out{};
    char* p = out.data();

    // prefix
    for (char c : prefix) *p++ = c;

    // nombre
    {
        char tmp[10];
        unsigned v = value;
        std::size_t i = 0;
        do {
            tmp[i++] = char('0' + (v % 10));
            v /= 10;
        } while (v);

        while (i--) *p++ = tmp[i];
    }

    // suffix
    for (char c : suffix) *p++ = c;

    // zéro terminal
    *p = '\0';

    return out;
}

// lambda constexpr évaluée immédiatement -> produit le buffer
inline constexpr auto CAN_BITRATE_STR_ARR = [] {
    return make_bitrate_array<CAN_BITRATE>();
}();

// vue pratique sur la chaîne
inline constexpr std::string_view CAN_BITRATE_STR{
    CAN_BITRATE_STR_ARR.data(),
    CAN_BITRATE_STR_ARR.size()
};

#define STR_(x) #x
#define STR(x)  STR_(x)
static void cmd_can(BaseSequentialStream *lchp, int ,const char* const [])
{
  constexpr bool fdframe = (CAN_BITRATE > 1000) or (CAN_BITRATE < 0);
  chprintf(lchp, "platform = %s version %d; %s; frame = %s; software version = %u.%u\r\n",
	   STR(PLATFORM), HW_VERSION,
	   CAN_BITRATE_STR.data(),
	   fdframe ? "CAN-FD" : "CAN-2.0",
	   SW_VERSION_MAJOR, SW_VERSION_MINOR);
}



using pGetFunc_t = uint32_t (*) (void);
using pSetFunc_t  = void (*) (uint32_t);


static void cmd_restart(BaseSequentialStream *lchp, int argc,const char* const argv[])
{
  (void) lchp;
  (void) argc;
  (void) argv;
  systemReset();
}

static void cmd_storage(BaseSequentialStream *lchp, int argc,const char* const argv[])
{
  (void) lchp;
  switch (argc) {
  case 0 : cmd_storage_list(); break;
  case 1 : cmd_storage_display(argv[0]); break;
  case 2 : cmd_storage_set(argv[0], argv[1]); break;
  default: {
    FixedString concat = argv[1];
    for (int i = 2; i < argc; i++) {
      concat += ' ';
      concat += argv[i];
    }
    cmd_storage_set(argv[0], concat.c_str());
  }
  }
}

static void cmd_uavparam(BaseSequentialStream *lchp, int argc,const char* const argv[])
{
  (void) lchp;
  switch (argc) {
  case 0 : cmd_storage_list(); break;
  case 1 : cmd_uavcan_storage_display(argv[0]); break;
  case 2 : cmd_uavcan_storage_set(argv[0], argv[1]); break;
  default: {
    FixedString concat = argv[1];
    for (int i = 2; i < argc; i++) {
      concat += ' ';
      concat += argv[i];
    }
    cmd_uavcan_storage_set(argv[0], concat);
    }
  }
}

/*
  
 */


/*===========================================================================*/
/* START OF PRIVATE SECTION  : DO NOT CHANGE ANYTHING BELOW THIS LINE        */
/*===========================================================================*/

/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/


#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2000)




#ifndef CONSOLE_DEV_USB
#define  CONSOLE_DEV_USB 0
#endif

#if CONSOLE_DEV_USB == 0
static const SerialConfig ftdiConfig =  {
  115200,
  0,
  USART_CR2_STOP1_BITS | USART_CR2_LINEN,
  0
};
#endif


#define MAX_CPU_INFO_ENTRIES 20

typedef struct _ThreadCpuInfo {
  float    ticks[MAX_CPU_INFO_ENTRIES];
  float    cpu[MAX_CPU_INFO_ENTRIES];
  float    totalTicks;
  float    totalISRTicks;
  _ThreadCpuInfo () {
    for (auto i=0; i< MAX_CPU_INFO_ENTRIES; i++) {
      ticks[i] = 0.0f;
      cpu[i] = -1.0f;
    }
    totalTicks = 0.0f;
    totalISRTicks = 0.0f;
  }
} ThreadCpuInfo ;
  
#if CH_DBG_STATISTICS
static void stampThreadCpuInfo (ThreadCpuInfo *ti);
static float stampThreadGetCpuPercent (const ThreadCpuInfo *ti, const uint32_t idx);
static float stampISRGetCpuPercent (const ThreadCpuInfo *ti);
#endif

static void cmd_uid(BaseSequentialStream *lchp, int argc,const char* const argv[]) {
  (void)argv;
  if (argc > 0) {
     chprintf(lchp, "Usage: uid\r\n");
    return;
  }

  for (uint32_t i=0; i< UniqProcessorIdLen; i++)
    chprintf(lchp, "[%x] ", UniqProcessorId[i]);
  chprintf(lchp, "\r\n");
}


static void cmd_mem(BaseSequentialStream *lchp, int argc,const char* const argv[]) {
  (void)argv;
  if (argc > 0) {
    chprintf(lchp, "Usage: mem\r\n");
    return;
  }

  chprintf(lchp, "core free memory : %u bytes\r\n", chCoreGetStatusX());

#if CH_HEAP_SIZE != 0
  chprintf(lchp, "heap free memory : %u bytes\r\n", getHeapFree());
  
  void * ptr1 = malloc_m (100);
  void * ptr2 = malloc_m (100);
  
  chprintf(lchp, "(2x) malloc_m(1000) = %p ;; %p\r\n", ptr1, ptr2);
  chprintf(lchp, "heap free memory : %d bytes\r\n", getHeapFree());
  
  free_m (ptr1);
  free_m (ptr2);

 
  
#endif
  
}



#if  CH_DBG_STATISTICS
static void cmd_threads(BaseSequentialStream *lchp, int argc,const char * const argv[]) {
  static const char *states[] = {CH_STATE_NAMES};
  thread_t *tp = chRegFirstThread();
  (void)argv;
  (void)argc;
  float totalTicks=0;
  float idleTicks=0;

  static ThreadCpuInfo threadCpuInfo;
  
  stampThreadCpuInfo (&threadCpuInfo);
  
  chprintf (lchp, "    addr    stack  frestk prio refs  state        time \t percent        name\r\n");
  uint32_t idx=0;
  do {
    chprintf (lchp, "%.8lx %.8lx %6lu %4lu %4lu %9s %9lu   %.2f%%    \t%s\r\n",
	      (uint32_t)tp, (uint32_t)tp->ctx.sp,
	      get_stack_free(tp),
	      (uint32_t)tp->hdr.pqueue.prio, (uint32_t)(tp->refs - 1),
	      states[tp->state],
	      (uint32_t)RTC2MS(STM32_SYSCLK, tp->stats.cumulative),
	      stampThreadGetCpuPercent (&threadCpuInfo, idx),
	      chRegGetThreadNameX(tp));

    totalTicks+= (float)tp->stats.cumulative;
    if (strcmp(chRegGetThreadNameX(tp), "idle") == 0)
    idleTicks = (float)tp->stats.cumulative;
    tp = chRegNextThread((thread_t *)tp);
    idx++;
  } while (tp != NULL);

  const float idlePercent = (idleTicks*100.f)/totalTicks;
  const float cpuPercent = 100.f - idlePercent;
  chprintf (lchp, "Interrupt Service Routine \t\t     %9lu   %.2f%%    \tISR\r\n",
	    (uint32_t)RTC2MS(STM32_SYSCLK,threadCpuInfo.totalISRTicks),
	    stampISRGetCpuPercent(&threadCpuInfo));
  chprintf (lchp, "\r\ncpu load = %.2f%%\r\n", cpuPercent);
}
#endif

static const ShellConfig shell_cfg1 = {
#if CONSOLE_DEV_USB == 0
  (BaseSequentialStream *) &CONSOLE_DEV_SD,
#else
  (BaseSequentialStream *) &SDU1,
#endif
  commands
};



void consoleInit (void)
{
  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * USBD1 : FS, USBD2 : HS
   */

#if CONSOLE_DEV_USB != 0
  usbSerialInit(&SDU1, &USBD1); 
  chp = (BaseSequentialStream *) &SDU1;
#else
  sdStart(&CONSOLE_DEV_SD, &ftdiConfig);
  chp = (BaseSequentialStream *) &CONSOLE_DEV_SD;
#endif
  /*
   * Shell manager initialization.
   */
  shellInit();
}


void consoleLaunch (void)
{
  thread_t *shelltp = NULL;

 
#if CONSOLE_DEV_USB != 0
  if (!shelltp) {
    while (usbGetDriver()->state != USB_ACTIVE) {
      chThdSleepMilliseconds(10);
    }
    
    // activate driver, giovani workaround
    chnGetTimeout(&SDU1, TIME_IMMEDIATE);
    while (!isUsbConnected()) {
      chThdSleepMilliseconds(10);
    }
    shelltp = shellCreate(&shell_cfg1, SHELL_WA_SIZE, NORMALPRIO - 1);
    //    palSetLine(LINE_USB_LED);
  } else if (shelltp && (chThdTerminatedX(shelltp))) {
    chThdRelease(shelltp);    /* Recovers memory of the previous shell.   */
    shelltp = NULL;           /* Triggers spawning of a new shell.        */
  }

#else // CONSOLE_DEV_USB == 0

   if (!shelltp) {
     shelltp = shellCreate(&shell_cfg1, SHELL_WA_SIZE, NORMALPRIO - 1);
   } else if (chThdTerminatedX(shelltp)) {
     chThdRelease(shelltp);    /* Recovers memory of the previous shell.   */
     shelltp = NULL;           /* Triggers spawning of a new shell.        */
   }
   chThdSleepMilliseconds(100);
   
#endif //CONSOLE_DEV_USB

}

namespace {
  struct OverloadDyn {
    void operator()(const ssize_t i, const frozen::string& name, Persistant::NoValue) const {
      DebugTrace("store %d '%s' has no value !!", i, name.data());
    }
    void operator()(const ssize_t i, const frozen::string& name, Persistant::Integer j) const {
      DebugTrace("store %d '%s'  is Integer = %lld", i, name.data(), j);
    }
    void operator()(const ssize_t i, const frozen::string& name, bool b) const {
      DebugTrace("store %d '%s'  is Bool = %d", i, name.data(), b);
    }
    void operator()(const ssize_t i, const frozen::string& name, float f) const {
      DebugTrace("store %d '%s'  is Double = %f", i, name.data(), f);
    }
    void operator()(const ssize_t i, const frozen::string& name, Persistant::StoredString *s) const {
      DebugTrace("store %d '%s'  is String = %s", i, name.data(),  s->c_str());
    }
  };

  void cmd_storage_list()
  {
    for (ssize_t i=0; i < Persistant::params_list_len; i++) {
      const frozen::string& paramName =  std::next(Persistant::frozenParameters.begin(), i)->first;
      std::visit([&](const auto& param_ptr) {
	OverloadDyn{}(i, paramName, param_ptr);  
      }, Persistant::Parameter::find(i).first);
    }
    DebugTrace("\n");
  }
  
  void cmd_storage_display(const char* key)
  {
    ssize_t index;
    if (auto opt = parse_value_int(key)) {
      index = opt.value();
      const auto& frozen_key =  std::next(frozenParameters.begin(), index)->first;
      key = frozen_key.data();
    } else {
      index = Persistant::Parameter::findIndex(key);
    }
    if (index < 0) {
      DebugTrace("ERROR : parameter %s not found", key);
    } else {
      std::visit([&](const auto& param_ptr) {
	OverloadDyn{}(index, frozen::string(key), param_ptr);
      }, Persistant::Parameter::find(index).first);
    }
  }

  void cmd_storage_display(ssize_t index)
  {
    if (index <= 0) {
      DebugTrace("ERROR : parameter index %d not valid", index);
    } else {
      const auto& key =  std::next(frozenParameters.begin(), index)->first;
      std::visit([&](const auto& param_ptr) {
	OverloadDyn{}(index, frozen::string(key), param_ptr);
      }, Persistant::Parameter::find(index).first);
    }
  }
  
  void cmd_storage_set(const char* key, const char* value)
  {
    ssize_t index;
    if (auto opt = parse_value_int(key)) {
      index = opt.value();
    } else {
      index = Persistant::Parameter::findIndex(key);
    }
    if (index < 0) {
      DebugTrace("ERROR : parameter %s not found", key);
    } else {
      const auto& p = Persistant::Parameter::find(index);
      Persistant::Parameter::set(p, value);
      Ressource::storage.store(index);
      cmd_storage_display(index);
      Ressource::storage.restore(index);
      cmd_storage_display(index);
    }
  }

  /*
    struct uavcan_protocol_param_GetSetRequest {
      uint16_t index;
      struct uavcan_protocol_param_Value value;
      struct { uint8_t len; uint8_t data[92]; }name;
    };

enum uavcan_protocol_param_Value_type_t {
    UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY,
    UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE,
    UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE,
    UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE,
    UAVCAN_PROTOCOL_PARAM_VALUE_STRING_VALUE,
};
    struct uavcan_protocol_param_Value {
      enum uavcan_protocol_param_Value_type_t union_tag;
      union {
        struct uavcan_protocol_param_Empty empty;
        int64_t integer_value;
        float real_value;
        uint8_t boolean_value;
      struct { uint8_t len; uint8_t data[128]; }string_value;
      };
    };

    enum uavcan_protocol_param_NumericValue_type_t {
    UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_EMPTY,
    UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE,
    UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_REAL_VALUE,
    };
    
    
    struct uavcan_protocol_param_NumericValue {
    enum uavcan_protocol_param_NumericValue_type_t union_tag;
    union {
    struct uavcan_protocol_param_Empty empty;
    int64_t integer_value;
    float real_value;
    };
    };
    
    struct uavcan_protocol_param_GetSetResponse {
      struct uavcan_protocol_param_Value value;
      struct uavcan_protocol_param_Value default_value;
      struct uavcan_protocol_param_NumericValue max_value;
      struct uavcan_protocol_param_NumericValue min_value;
      struct { uint8_t len; uint8_t data[92]; }name;
    };


  */
  
  void cmd_uavcan_storage_display(etl::string_view key)
  {
    int index = -1;
    if (auto opt = parse_value_int(key)) {
      index = opt.value();
    } 
    uavcan_protocol_param_GetSetRequest req = {
      .index = static_cast<std::uint16_t>(std::clamp(index, 0,
					  static_cast<int>(std::numeric_limits<std::uint16_t>::max()))),
      .value = {
	.union_tag =  UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY,
	.integer_value = 0
      },
      .name = {
	.len = static_cast<uint8_t>(index == 0 ? strlen(key.data()) : 0),
	.data = 0
      }
    };
    if (index < 0)
      memcpy(req.name.data, key.data(), strlen(key.data()));
    
    uavcan_protocol_param_GetSetResponse resp;
    getSetResponse(req, resp);
    
    FixedString respStr;
    respStr << resp;
    DebugTrace("resp = %s", respStr.c_str());
  }

  
  void cmd_uavcan_storage_set(etl::string_view key, etl::string_view valuestr)
  {
    uint16_t index = 0;
    if (auto opt = parse_value_int(key)) {
      index = opt.value();
    } 
    uavcan_protocol_param_GetSetRequest req = {
      .index = index,
      .value = {
	.union_tag =  UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY,
	.integer_value = 0
      },
      .name = {
	.len = static_cast<uint8_t>(index == 0 ? strlen(key.data()) : 0),
	.data = 0
      }
    };
    if (index == 0)
      memcpy(req.name.data, key.data(), strlen(key.data()));
    
    const Value value = parse_value(valuestr);
    req.value.union_tag  = static_cast<uavcan_protocol_param_Value_type_t>(value.index());
    switch (value.index()) {
    case 0:
      DebugTrace("empty"); break;
    case 1:
      req.value.integer_value = std::get<int64_t>(value);
      break;
    case 2:
      req.value.real_value = std::get<float>(value);
      break;
    case 3:
      req.value.boolean_value = std::get<bool>(value);
      break;
    case 4:
      req.value.string_value.len = std::get<FixedString>(value).length();
      strcpy(reinterpret_cast<char *>(req.value.string_value.data),
	     std::get<FixedString>(value).c_str());
      break;
    default:
      DebugTrace("OOPS");
    }
    
    uavcan_protocol_param_GetSetResponse resp;
    const auto [stoIdx, _] = getSetResponse(req, resp);
    Ressource::storage.store(stoIdx);
    
    FixedString respStr;
    respStr << resp;
    DebugTrace("resp = %s", respStr.c_str());
  }


  Value parse_value(etl::string_view input) {
    // 1. Trim left/right
    while (!input.empty() && std::isspace(input.front())) input.remove_prefix(1);
    while (!input.empty() && std::isspace(input.back())) input.remove_suffix(1);
    
    // 2. Boolean check
    if (input.size() == 1) {
        const char c = input[0];
        if (c == 't' || c == 'T' || c == 'v' || c == 'V') return true;
    }

    // 3. Try int64_t
    int64_t i;
    const auto [p1, ec1] = std::from_chars(input.begin(), input.end(), i);
    if (ec1 == std::errc() && p1 == input.end()) return i;

    // 4. Try float
    float f;
    const auto [p2, ec2] = std::from_chars(input.begin(), input.end(), f);
    if (ec2 == std::errc() && p2 == input.end()) return f;

    // 5. Default to string
    return FixedString{input};
  }


  std::optional<int> parse_value_int(etl::string_view input)
  {
    const Value v = parse_value(input);
    if (std::holds_alternative<int64_t>(v)) {
      return std::get<int64_t>(v);
    } else {
      return {};
    }
  }


  FixedString &  appendFixed(FixedString &str, const uavcan_protocol_param_NumericValue &val)
  {
    FixedString added;
    switch (val.union_tag) {
    case UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_EMPTY :
      added = "{}";
      break;
    case UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE :
      snprintf(added.begin(), added.size(), "%lld", val.integer_value);
      break;
    case UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_REAL_VALUE :
      snprintf(added.begin(), added.size(), "%f", val.real_value);
      break;
      }
    added.uninitialized_resize(strlen(added.c_str()));
    str += added;
    return str;
  }
  
  FixedString &  appendFixed(FixedString &str, const uavcan_protocol_param_Value &val)
  {
    FixedString added;
    switch (val.union_tag) {
    case UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY :
      added = "{}";
      break;
    case UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE :
       snprintf(added.begin(), added.size(), "%lld", val.integer_value);
      break;
    case UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE :
      snprintf(added.begin(), added.size(), "%f", val.real_value);
      break;
     case UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE :
       added =  (val.boolean_value == true) ? "TRUE" : "FALSE";
       break;
     case UAVCAN_PROTOCOL_PARAM_VALUE_STRING_VALUE :
       added = etl::string_view(reinterpret_cast<const char *>(val.string_value.data),
				val.string_value.len);
      break;
    }
    added.uninitialized_resize(strlen(added.c_str()));
    str += added;
    return str;
  }

  FixedString &  appendFixed(FixedString &str, const char *val)
  {
    str += val;
    return str;
  }

  FixedString &  appendFixed(FixedString &str, const uint8_t *ptr, size_t len)
  {
    str += FixedString(reinterpret_cast<const char*>(ptr), len);
    return str;
  }

  FixedString &  appendFixed(FixedString &str, const uavcan_protocol_param_GetSetResponse &resp)
  {
    appendFixed(str, resp.name.data,  resp.name.len);
    str << " max = " << resp.max_value
	<< " min = " << resp.min_value
	<< " default = " << resp.default_value
	<< " value = "   << resp.value;
    return str;
  }

  
}

#if CH_DBG_STATISTICS
static void stampThreadCpuInfo (ThreadCpuInfo *ti)
{
  const thread_t *tp =  chRegFirstThread();
  uint32_t idx=0;
  
  ti->totalTicks =0;
  do {
    ti->ticks[idx] = (float) tp->stats.cumulative;
    ti->totalTicks += ti->ticks[idx];
    tp = chRegNextThread ((thread_t *)tp);
    idx++;
  } while ((tp != NULL) && (idx < MAX_CPU_INFO_ENTRIES));
  ti->totalISRTicks = currcore->kernel_stats.m_crit_isr.cumulative;
  ti->totalTicks += ti->totalISRTicks;
  tp =  chRegFirstThread();
  idx=0;
  do {
    ti->cpu[idx] =  (ti->ticks[idx]*100.f) / ti->totalTicks;
    tp = chRegNextThread ((thread_t *)tp);
    idx++;
  } while ((tp != NULL) && (idx < MAX_CPU_INFO_ENTRIES));
}

static float stampThreadGetCpuPercent (const ThreadCpuInfo *ti, const uint32_t idx)
{
  if (idx >= MAX_CPU_INFO_ENTRIES) 
    return -1.f;

  return ti->cpu[idx];
}

static float stampISRGetCpuPercent (const ThreadCpuInfo *ti)
{
  return ti->totalISRTicks * 100.0f / ti->totalTicks;
}
#endif // CH_DBG_STATISTICS
#endif // CONSOLE_DEV_SD
#endif // TRACE
