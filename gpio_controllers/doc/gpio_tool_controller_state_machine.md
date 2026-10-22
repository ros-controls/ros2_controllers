State machine of the `gpio_tool_controller`.
Solid arrows are transitions, dashed arrows are requests from the ROS 2 interface.
Service and action names use the default values of `engaged.name` (`engaged`) and `disengaged.name` (`disengaged`).

```mermaid
flowchart TD
    ENG[/"<b>Engage / disengage request</b><br/>services ~/engaged, ~/disengaged<br/>or action ~/engaged_disengaged (goal.engage)"/]
    REC[/"<b>Reconfigure request</b><br/>service or action ~/reconfigure (config_name)<br/>only if disengaged or enable_config_engaged"/]
    RST[/"<b>Reset request</b><br/>service ~/reset_halted"/]

    IDLE("<b>IDLE</b><br/>checks tool state and configuration every cycle")
    HALTED("<b>HALTED</b><br/>commands frozen, new requests rejected")

    subgraph SEQ ["Transition sequence"]
        SBC["<b>SET_BEFORE_COMMAND</b><br/>set preparatory commands"]
        CBC["<b>CHECK_BEFORE_COMMAND</b><br/>wait for preparatory states"]
        SC["<b>SET_COMMAND</b><br/>set main commands"]
        CC["<b>CHECK_COMMAND</b><br/>wait for target state<br/>(engage: any of possible_engaged_states)"]
        SAC["<b>SET_AFTER_COMMAND</b><br/>set post-action commands"]
        CAC["<b>CHECK_AFTER_COMMAND</b><br/>wait for post-action states"]
        SBC --> CBC --> SC --> CC --> SAC --> CAC
    end

    ENG -.-> IDLE
    ENG -.->|"opposite request<br/>restarts the sequence"| SBC
    REC -.-> IDLE
    RST -.-> HALTED

    IDLE -->|request accepted| SBC
    CAC -->|success| IDLE
    SEQ -->|"CHECK_* timeout,<br/>write failure,<br/>or cancel"| HALTED
    IDLE -->|"tool state or configuration<br/>cannot be determined"| HALTED
    HALTED -->|"reset: tool state is read<br/>from the hardware"| IDLE
```
