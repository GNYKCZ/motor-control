"""标定采样相位的独立有理数模型和生成配置闭包；不证明外部放大器建立或实测WCET。"""
from fractions import Fraction as F
import hashlib
import json
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[2]
FW = ROOT / "firmware/stm32f103"
paths = [FW / n for n in ("m1_sampling.ioc", "Core/Src/tim.c", "Core/Src/adc.c",
         "board/board_io.c", "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c")]
ioc, tim, adc, board, hal = [p.read_text(encoding="utf-8") for p in paths]
assert "htim1.Init.Prescaler = 0;" in tim and "htim1.Init.Period = 1800;" in tim
assert "htim1.Init.RepetitionCounter = 39;" in tim
tim2 = tim[tim.index("void MX_TIM2_Init"):tim.index("void MX_TIM3_Init")]
tim1 = tim[tim.index("void MX_TIM1_Init"):tim.index("void MX_TIM2_Init")]
assert "sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;" in tim1
assert "sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;" in tim2
assert "sSlaveConfig.InputTrigger = TIM_TS_ITR0;" in tim2
assert "htim2.Init.Prescaler = 71;" in tim2
assert "sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF;" in tim2
assert re.search(r"sConfigOC.Pulse = 175;.*?HAL_TIM_OC_ConfigChannel.*?TIM_CHANNEL_1", tim2, re.S)
assert re.search(r"sConfigOC.OCMode = TIM_OCMODE_PWM2;\s+sConfigOC.Pulse = 225;.*?HAL_TIM_PWM_ConfigChannel.*?TIM_CHANNEL_2", tim2, re.S)
assert "sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T2_TRGO;" in adc
assert "ADC_SAMPLETIME_7CYCLES_5" in adc and "ADC_DUALMODE_INJECSIMULT" in adc
assert "ADC1.ExternalTrigInjecConv=ADC_EXTERNALTRIGINJECCONV_T2_TRGO" in ioc
assert board.count("TIM1->EGR=TIM_EGR_UG;")==1 and "TIM2->EGR" not in board
start=board.index("bool board_start_sampling")
assert start < board.index("TIM1->EGR=TIM_EGR_UG;") < board.index("HAL_ADCEx_InjectedStart(&hadc2)",start)
configuration = hal[hal.index("HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel"):]
assert configuration.index("TIM_OC2_SetConfig(htim->Instance, sConfig)") < configuration.index("htim->Instance->CCMR1 |= TIM_CCMR1_OC2PE")
# HAL先写CCR2，再打开预装，初始OC2REF不依赖运行期手工UG。
assert "TIMx->CCR2 = OC_Config->Pulse;" in hal

counter = F(72000000, 1)
period = F(2 * 1800, counter)
update = 20 * period  # 40次半周期事件，RCR39。
assert period==F(1,20000) and update==F(1,1000)
spi_start = F(175, 1000000)
spi_duration = F(24 * 64, 72000000)
adc_start = F(225, 1000000)
aperture = F(15, 2 * 12000000)
adc_total = F(20, 12000000)
assert spi_start + spi_duration < adc_start
assert (adc_start + adc_total - spi_start) * 1000000 < 200
assert F(24 * 64, 72000000) * 1000000 < 40

# 从首个独立TIM2周期到8个自然更新周期，均须一帧SSI先完成，再采ADC。
events = []
for index in range(8):
    base = index * update
    events.append((base + spi_start, base + spi_start + spi_duration, base + adc_start))
    assert events[-1][1] < events[-1][2]
    if index:
        assert events[-1][2] - events[-2][2] == update
        assert events[-2][2] + adc_total < events[-1][0]  # 前次ADC拉高CS在下次SSI之前。

# 新单U只允许900/1800，175/225us均位于载波顶部；ADC全程在共同低电平。
carrier_adc=adc_start % period
fall=F(900,counter);rise=period-fall
assert carrier_adc==F(25,1000000)
assert fall < carrier_adc and carrier_adc+adc_total < rise
lead=carrier_adc-fall;tail=rise-carrier_adc-adc_total
result = {"result": "PASS", "head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
          "pwm_period_us": float(period * 1000000), "raw_period_us": float(update * 1000000),
          "adc_aperture_us": float(aperture * 1000000), "adc_total_us": float(adc_total * 1000000),
          "nominal_spi_age_at_adc_completion_us": float((adc_start + adc_total - spi_start) * 1000000),
          "nominal_stable_lead_us_at_900": float(lead * 1000000),
          "nominal_stable_tail_us_at_900": float(tail * 1000000),
          "limits": "理想计数/生成配置模型；未包含实际触发/IRQ/放大器建立偏差，非电气有效性证明；未证明新profile实际电气安全",
          "hardware": "NOT RUN", "input_sha256": {str(p.relative_to(ROOT)).replace('\\', '/'): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}}
out = ROOT / "build/current_angle"
out.mkdir(parents=True, exist_ok=True)
(out / "timing_model.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print("PASS calibration timing: generated TIM2/ADC route, first/steady frame sequence, exact PWM/SSI/ADC geometry; external analog NOT RUN")
