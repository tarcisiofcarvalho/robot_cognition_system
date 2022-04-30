
Vue.component('component_target_distance', {
    props: ['target_distance', 'target_x', 'target_y'],
    template: '<p>Target distance: {{ target_distance }} </p>'
});


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
        process_error: false,
        target_distance: '',
        target_x: '',
        target_y: ''
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
                    this.target_distance = response.data.target_distance,
                    this.target_x = response.data.target_x,
                    this.target_y = response.data.target_y
                    // console.log(response.data)
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
            this.laser_pan = this.laser_pan - 1;
            this.process_laser();
        },

        plus_pan(){
            this.laser_pan = this.laser_pan + 1;
            this.process_laser();
        },

        minus_tilt(){
            this.laser_tilt = this.laser_tilt - 1;
            this.process_laser();
        }, 

        plus_tilt(){
            this.laser_tilt = this.laser_tilt + 1;
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
                        console.log(response.status);
                        if(this.process_interrupted){
                            this.go_achieved = false;
                        }else{
                            this.go_achieved = true;
                        }
                        this.moving = false;
                        this.process_error = false;
                    }else{
                        this.go_achieved = false;
                        this.moving = false;
                        this.process_error = true;
                    }
                }
            )
        },

        stop(){
            this.stopping = true;
            axios
            .post('/base_stop')
            .then(
                response => {
                    if(response.status == 200){
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
    },
    template:`
        <div>
            <div class="w3-container w3-red w3-center" v-if="passage_condition=='unsafe'">
                <p>Unsafe Passage Condition {{ passage_condition }}</p>
            </div>
            <div class="w3-container w3-green w3-center" v-else>
                <p>Safe Passage Condition {{ passage_condition }}</p>
            </div> 
            <div class="w3-container w3-bar w3-center w3-marging">
                <p>Laser Pan Degree</p>
                <button class="w3-button w3-blue w3-circle w3-smal" v-bind:class="{'w3-disabled': moving}" v-on:click="minus_pan()">-</button>
                <input v-model="laser_pan"/>
                <button class="w3-button w3-green w3-circle w3-smal" v-bind:class="{'w3-disabled': moving}" v-on:click="plus_pan()">+</button>
            </div>
            <div class="w3-container w3-bar w3-center w3-marging">
                <p>Laser Tilt Degree</p>
                <button class="w3-button w3-blue w3-circle w3-smal" v-bind:class="{'w3-disabled': moving}" v-on:click="minus_tilt()">-</button>
                <input v-model="laser_tilt"/>
                <button class="w3-button w3-green w3-circle w3-smal" v-bind:class="{'w3-disabled': moving}" v-on:click="plus_tilt()">+</button>
            </div>
            <component_target_distance 
                v-bind:target_distance="target_distance"
                v-bind:target_x="target_x"
                v-bind:target_y="target_y"
                >
            </component_target_distance>                 
            <p v-if="passage_condition=='safe' && moving==false && process_error==false"><button class="w3-button w3-block w3-green" v-on:click="go();">Click here to Go!</button></p>
            <div class="w3-center w3-panel" v-if="moving">
                <div class="loader"></div>
                <span>Moving...</span>
            </div>
            <p v-if="moving">
                <button class="w3-button w3-block w3-red" v-on:click="stop()">Emergency Stop</button>
            </p> 
            <div class="w3-panel w3-green" v-if="go_achieved">
                <h3>Success!</h3>
                <p>Target achieved.</p>
            </div>
            <div class="w3-panel w3-blue" v-if="stopping">
                <h3>Warning!</h3>
                <p>Stopping the Wheeledchair...</p>
            </div>                  
            <div class="w3-panel w3-blue" v-if="process_interrupted">
                <h3>Info!</h3>
                <p>Wheeledchair stopped!</p>
            </div>      
            <div class="w3-panel w3-blue" v-if="process_error">
                <h3>Warning!</h3>
                <p>Something went wrong!</p>
            </div>  
        </div>                              
    `
})