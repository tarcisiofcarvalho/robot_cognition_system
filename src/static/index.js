var app = new Vue({
    el: "#app",
    data:{
        passage_condition: "x",
        laser_pan: 0,
        laser_tilt: 0
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
        }        
    }

})