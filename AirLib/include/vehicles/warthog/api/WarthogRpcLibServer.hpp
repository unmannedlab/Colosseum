// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_WarthogRpcLibServer_hpp
#define air_WarthogRpcLibServer_hpp

#ifndef AIRLIB_NO_RPC

#include "common/Common.hpp"
#include <functional>
#include "api/RpcLibServerBase.hpp"
#include "vehicles/warthog/api/WarthogApiBase.hpp"

namespace msr
{
namespace airlib
{

    class WarthogRpcLibServer : public RpcLibServerBase
    {
    public:
        WarthogRpcLibServer(ApiProvider* api_provider, string server_address, uint16_t port = RpcLibPort);
        virtual ~WarthogRpcLibServer();

    protected:
        virtual WarthogApiBase* getVehicleApi(const std::string& vehicle_name) override
        {
            return static_cast<WarthogApiBase*>(RpcLibServerBase::getVehicleApi(vehicle_name));
        }
    };

#endif
}
} //namespace
#endif