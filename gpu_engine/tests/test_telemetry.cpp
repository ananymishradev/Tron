#include "app/telemetry.h"
#include "test_util.h"

int main() {
  History history;
  for (int i = 0; i < 10; ++i) history.push(static_cast<float>(i));
  CHECK(history.count() == 10);
  CHECK_NEAR(history.at(0), 0.0f, 1e-6f);
  CHECK_NEAR(history.at(9), 9.0f, 1e-6f);
  CHECK_NEAR(history.percentile(0.5f), 4.0f, 1e-6f);
  Telemetry telemetry;
  telemetry.frame = 42;
  telemetry.sim_ms = 1.25;
  telemetry.stats.firstNonFinite = 123;
  telemetry.log_path = "a\"b.csv";
  const std::string json = telemetry_json(telemetry);
  CHECK(json.find("\"frame\":42") != std::string::npos);
  CHECK(json.find("\"sim_ms\":1.25") != std::string::npos);
  CHECK(json.find("\"nan\":123") != std::string::npos);
  CHECK(json.find("a\\\"b.csv") != std::string::npos);
  return 0;
}
