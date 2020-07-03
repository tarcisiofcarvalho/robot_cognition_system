var app = new Vue({
    el: "#app",
    data:{
        passage_condition: "x"
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
        }
    }

})