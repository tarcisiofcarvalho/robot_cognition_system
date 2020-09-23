var app = new Vue({
    el: "#app",
    data:{
        passage_condition: "safe",
        laser_pan: 0,
        laser_tilt: 0,
        moving: false,
        process_interrupted: false,
        go_achieved: false,
        stopping: false,
        process_error: false
    },
    created(){
        this.interval = setInterval(() => this.getPassageCondition(),1000);
    },

    methods:{
        getPassageCondition(){
            axios
            .get('/passage_condition')
            .then(
                response => (
                    this.passage_condition = response.data.condition,
                    console.log(response.data.condition)
                )
            )
        },

        process_laser(){
            this.reset_flags();
            var body ={
                "pan_value": this.laser_pan,
                "tilt_value": this.laser_tilt
            }
            axios
            .post('/laser_move',data=body)
            .then(
                response => (
                    console.log(response)
                )
            )
        },

        reset_flags(){
            this.moving = false;
            this.process_interrupted = false;
            this.go_achieved = false;
            this.stopping = false;
        },
        minus_pan(){
            this.laser_pan = this.laser_pan - 5;
            this.process_laser();
        },
        plus_pan(){
            this.laser_pan = this.laser_pan + 5;
            this.process_laser();
        },
        minus_tilt(){
            this.laser_tilt = this.laser_tilt - 5;
            this.process_laser();
        },                
        plus_tilt(){
            this.laser_tilt = this.laser_tilt + 5;
            this.process_laser();
        },
        go(){
            this.reset_flags();
            this.moving = true;
            var body ={
                "pan_value": this.laser_pan,
                "tilt_value": this.laser_tilt
            }
            axios
            .post('/base_move',data=body)
            .then(response => {
                    if(response.status === 200){
                        console.log("checkpoint 4");
                        console.log(response.status);
                        if(this.process_interrupted){
                            this.go_achieved = false;
                        }else{
                            this.go_achieved = true;
                        }
                        this.moving = false;
                        this.process_error = false;
                    }else{
                        console.log("checkpoint 5");
                        this.go_achieved = false;
                        this.moving = false;
                        this.process_error = true;
                    }
                }
            )
        },
        stop(){
            console.log("checkpoint 3");
            this.stopping = true;
            axios
            .post('/base_stop')
            .then(
                response => {
                    console.log("checkpoint 3.1");
                    console.log(response);
                    if(response.status == 200){
                        console.log("checkpoint 3.2");
                        this.stopping = false;
                        this.go_achieved = false;
                        this.moving = false;
                        this.process_interrupted = true;
                    }else{
                        this.process_error = true;
                        this.stopping = false;
                        this.go_achieved = false;
                        this.moving = false;
                        this.process_interrupted = false;
                    }
                }
                    
                
            )
        }       
    }

})