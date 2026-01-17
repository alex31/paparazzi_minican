/**
 * @file deviceResource.hpp
 * @brief Shared persistent storage handle for device resources.
 */
#include "MFS.hpp"
#include "UAVCAN/persistantParam.hpp"
#include "UAVCAN/persistantStorage.hpp"

/*
#                 _____                                                                    
#                |  __ \                                                                   
#                | |__) |   ___   ___    ___     ___    _   _   _ __    ___    ___         
#                |  _  /   / _ \ / __|  / __|   / _ \  | | | | | '__|  / __|  / _ \        
#                | | \ \  |  __/ \__ \  \__ \  | (_) | | |_| | | |    | (__  |  __/        
#                |_|  \_\  \___| |___/  |___/   \___/   \__,_| |_|     \___|  \___|        
*/
/** @brief Namespace containing device resource singletons. */
namespace Ressource {
  /** @brief Persistent storage back-end used by roles. */
  extern  Persistant::Storage storage;
}
