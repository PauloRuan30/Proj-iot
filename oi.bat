@echo off
set "root=sistema-monitoramento-iot"

rem Criar diretórios
mkdir "%root%\backend\api-gateway\src\main\java\com\projeto\gateway\config"
mkdir "%root%\backend\api-gateway\src\main\java\com\projeto\gateway\filter"
mkdir "%root%\backend\api-gateway\src\main\resources"
mkdir "%root%\backend\api-gateway\src\test"
mkdir "%root%\backend\auth-service\src\main\java\com\projeto\auth\config"
mkdir "%root%\backend\auth-service\src\main\java\com\projeto\auth\controller"
mkdir "%root%\backend\auth-service\src\main\java\com\projeto\auth\model"
mkdir "%root%\backend\auth-service\src\main\java\com\projeto\auth\repository"
mkdir "%root%\backend\auth-service\src\main\java\com\projeto\auth\service"
mkdir "%root%\backend\auth-service\src\main\resources\db\migration"
mkdir "%root%\backend\auth-service\src\test"
mkdir "%root%\backend\data-collector-service\src\main\java\com\projeto\datacollector\config"
mkdir "%root%\backend\data-collector-service\src\main\java\com\projeto\datacollector\controller"
mkdir "%root%\backend\data-collector-service\src\main\java\com\projeto\datacollector\grpc\server"
mkdir "%root%\backend\data-collector-service\src\main\java\com\projeto\datacollector\grpc\client"
mkdir "%root%\backend\data-collector-service\src\main\java\com\projeto\datacollector\model"
mkdir "%root%\backend\data-collector-service\src\main\java\com\projeto\datacollector\service"
mkdir "%root%\backend\data-collector-service\src\main\resources\proto"
mkdir "%root%\backend\data-collector-service\src\test"
mkdir "%root%\backend\processing-service\src\main\java\com\projeto\processing\config"
mkdir "%root%\backend\processing-service\src\main\java\com\projeto\processing\controller"
mkdir "%root%\backend\processing-service\src\main\java\com\projeto\processing\listener"
mkdir "%root%\backend\processing-service\src\main\java\com\projeto\processing\model"
mkdir "%root%\backend\processing-service\src\main\java\com\projeto\processing\repository"
mkdir "%root%\backend\processing-service\src\main\java\com\projeto\processing\service"
mkdir "%root%\backend\processing-service\src\test"
mkdir "%root%\backend\device-management-service\src\main\java\com\projeto\devicemanagement\config"
mkdir "%root%\backend\device-management-service\src\main\java\com\projeto\devicemanagement\controller"
mkdir "%root%\backend\device-management-service\src\main\java\com\projeto\devicemanagement\grpc"
mkdir "%root%\backend\device-management-service\src\main\java\com\projeto\devicemanagement\model"
mkdir "%root%\backend\device-management-service\src\main\java\com\projeto\devicemanagement\repository"
mkdir "%root%\backend\device-management-service\src\main\java\com\projeto\devicemanagement\service"
mkdir "%root%\backend\device-management-service\src\main\resources\db\migration"
mkdir "%root%\backend\device-management-service\src\test"
mkdir "%root%\backend\common-lib\src\main\java\com\projeto\common\dto"
mkdir "%root%\backend\common-lib\src\main\java\com\projeto\common\exception"
mkdir "%root%\backend\common-lib\src\main\java\com\projeto\common\util"
mkdir "%root%\backend\common-lib\src\test"

rem Frontend
mkdir "%root%\frontend\public"
mkdir "%root%\frontend\src\assets"
mkdir "%root%\frontend\src\components\common"
mkdir "%root%\frontend\src\components\dashboard"
mkdir "%root%\frontend\src\components\devices"
mkdir "%root%\frontend\src\components\alerts"
mkdir "%root%\frontend\src\components\settings"
mkdir "%root%\frontend\src\contexts"
mkdir "%root%\frontend\src\hooks"
mkdir "%root%\frontend\src\pages"
mkdir "%root%\frontend\src\services"
mkdir "%root%\frontend\src\utils"

rem IoT
mkdir "%root%\iot-devices\esp32\mqtt_client"
mkdir "%root%\iot-devices\esp32\sensor_modules"
mkdir "%root%\iot-devices\esp32\libraries"
mkdir "%root%\iot-devices\simulator\src\main\java\com\projeto\simulator\config"
mkdir "%root%\iot-devices\simulator\src\main\java\com\projeto\simulator\device"
mkdir "%root%\iot-devices\simulator\src\main\java\com\projeto\simulator\sensor"
mkdir "%root%\iot-devices\simulator\src\main\resources"

rem Kubernetes
mkdir "%root%\k8s\base\api-gateway"
mkdir "%root%\k8s\base\auth-service"
mkdir "%root%\k8s\base\data-collector-service"
mkdir "%root%\k8s\base\processing-service"
mkdir "%root%\k8s\base\device-management-service"
mkdir "%root%\k8s\base\database\postgres"
mkdir "%root%\k8s\base\database\mongodb"
mkdir "%root%\k8s\base\rabbitmq"
mkdir "%root%\k8s\base\monitoring\prometheus"
mkdir "%root%\k8s\base\monitoring\grafana"
mkdir "%root%\k8s\base\logging\elk"
mkdir "%root%\k8s\dev"
mkdir "%root%\k8s\prod"

rem Docker
mkdir "%root%\docker\api-gateway"
mkdir "%root%\docker\auth-service"
mkdir "%root%\docker\data-collector-service"
mkdir "%root%\docker\processing-service"
mkdir "%root%\docker\device-management-service"
mkdir "%root%\docker\frontend"

rem Docs
mkdir "%root%\docs\architecture\diagrams"
mkdir "%root%\docs\api"
mkdir "%root%\docs\deployment"
mkdir "%root%\docs\user-manual"

rem Scripts e GitHub Actions
mkdir "%root%\.github\workflows"
mkdir "%root%\scripts"

rem Criar arquivos vazios
type nul > "%root%\README.md"
type nul > "%root%\.gitignore"
type nul > "%root%\LICENSE"
type nul > "%root%\.github\workflows\ci-backend.yml"
type nul > "%root%\.github\workflows\ci-frontend.yml"
type nul > "%root%\.github\workflows\ci-infra.yml"

echo Estrutura criada com sucesso!
pause