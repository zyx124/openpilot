#!/usr/bin/env python3
from selfdrive.controls.behaviord import PARAMS

def generate_slider_code():
    cpp_code = 'BehaviorPanel::BehaviorPanel(SettingsWindow *parent) : ListWidget(parent){\n\n  // Add sliders here\n  // name, label, units, min, max, default, setter function\n  std::vector<SliderDefinition> slider_defs{\n'
    
    for param, info in PARAMS.items():
        min_val, max_val = info["range"]
        default_val = info["default"]
        label = info["label"]
        units = info["units"]
        setter_func = f'[](cereal::Behavior::Builder &behavior, double value) {{\n        behavior.set{param}(static_cast<float>(value));\n      }}'
        cpp_code += f'    {{"{param}", tr("{label}"), "{units}", {min_val}, {max_val}, {default_val},\n      {setter_func}\n    }},\n'
    
    cpp_code += '  };\n\n'
    
    return cpp_code

def generate_param_code():
    cpp_code = ''
    
    for param in PARAMS.keys():
        cpp_code += f'    {{"{param}", PERSISTENT}},\n'
    
    return cpp_code

def generate_log_code():
    cpp_code = 'struct Behavior {\n'
    
    for i, param in enumerate(PARAMS.keys()):
        cpp_code += f'  {param} @{i} :Float32;\n'
    
    cpp_code += '};'
    
    return cpp_code


if __name__ == "__main__":
    print("paste the output of this into selfdrive/ui/qt/settings.cc")
    print(generate_slider_code())
    print("paste the output of this into common/params.cc")
    print(generate_param_code())
    print("paste the output of this into cereal/log.capnp")
    print(generate_log_code())
    



