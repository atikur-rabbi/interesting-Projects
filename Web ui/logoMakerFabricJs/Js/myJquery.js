
//github
if (document.addEventListener) {
    document.addEventListener('contextmenu', function(e) {
        alert("You've tried to open context menu"); //here you draw your own menu
        e.preventDefault();
    }, false);
} else {
    document.attachEvent('oncontextmenu', function() {
        alert("You've tried to open context menu");
        window.event.returnValue = false;
    });
}

//  Downloading Image
$('#saveImg').on('click.js',saveImg);
function saveImg(){    
    downloadFabric(canvas,'<file name>');
    }

    function download(url,name){
        // make the link. set the href and download. emulate dom click
          $('<a>').attr({href:url,download:name})[0].click();
        }
    function downloadFabric(canvas,name){
        //  convert the canvas to a data url and download it.
          download(canvas.toDataURL(),name+'.png');
        }
fabric.StaticCanvas.prototype.clear = function () {
    this._objects.length = 0;
    this.backgroundImage = null;
    this.overlayImage = null;
    this.backgroundColor = '';
    this.overlayColor = '';
    if (this._hasITextHandlers) {
        this.off('mouse:up', this._mouseUpITextHandler);
        this._iTextInstances = null;
        this._hasITextHandlers = false;
    }
    this.clearContext(this.contextContainer);
    this.fire('canvas:cleared');
    this.renderOnAddRemove && this.requestRenderAll();
    return this;
    };

// UI Color box
$(".box-BGcolor").click(function(){
    $(".UIColorBox").toggle(1);
})
$("#new").click(function(){
    
    if(confirm("Are you sure you want to discard editing and create a new file?")==true){
        var canvas = new fabric.Canvas('myCanvas');
        canvas.backgroundColor="white";
        canvas.setHeight(500);
        canvas.setWidth(1150);
        canvas.forEachObject(function(o){ canvas.remove(o) });
        $(".box-BGcolor").css("background-color","white");
        canvas.renderAll();
    }
    else{
        alert("please save the file first")
    }
   
})
// Changing Background
$(".box").click(function(){
    var color= $(this).css("background-color");
    canvas.backgroundColor=color;
    canvas.renderAll();
    $(".box-BGcolor").css("background-color",color);
});

// toggleButton function and implementation
toggleButton("text","textBut");
toggleButton("symbol","symbBut");
toggleButton("shape","shapeBut");
toggleButton("img","imgBut");
function toggleButton(style,button){
    var style="."+style;
    var button="."+button;
    $(button).click(function(){
    $(".tool-box").not($(style)).hide(1);
    $(style).toggle(1);});
};
// end of function

// setting height and width of canvas
var canvas = new fabric.Canvas('myCanvas');
canvas.backgroundColor="white";
canvas.setHeight(500);
canvas.setWidth(1150);
canvas.renderAll();

// Changing the font Size
$("#fontSize").change(onFontSizeChanged);
function onFontSizeChanged() {
    var newFontSize = $(this).val();
  var activeObject = canvas.getActiveObject();
  if (activeObject.setSelectionStyles && activeObject.isEditing) {
          var style = { };
          style["fontSize"] = newFontSize;
          activeObject.setSelectionStyles(style);
      } else {
          activeObject["fontSize"] = newFontSize;
      }
    
  canvas.renderAll();
  
}

// Changing font-Family
$('#FontStyleNumber').change(function () {
    debugger;
    var mFont = $(this).val();
    var tObj = canvas.getActiveObject();
    tObj.set({
        fontFamily: mFont
    });
    canvas.renderAll();
});

// Making backspace delete objects
$('html').keyup(function(e){
    if(e.keyCode == 8){
    canvas.getActiveObject().remove();
    if(canvas.getActiveGroup()){
        canvas.getActiveGroup().forEachObject(function(o){ canvas.remove(o) });
        canvas.discardActiveGroup().renderAll();
      } else {
        canvas.remove(canvas.getActiveObject());
      }

    }    })  ;


// Adding shapes
    // Adding circle
$(".fa-circle").click(function(){
    canvas.add(new fabric.Circle({ radius: 30, fill: '#f55', top: 100, left: 100 }));
})
// Adding Triangle
$(".fa-caret-up").click(function(){
    canvas.add( new fabric.Triangle({ top: 300, left: 210, width: 100, height: 100, fill: 'blue' }))
})
// Adding Square
$(".fa-stop").click(function(){
   canvas.add( new fabric.Rect({ top: 100, left: 400, width: 50, height: 50, fill: '#f55' }))
})
// changing colors of shapes
$(".box2").click(function(){
    var color= $(this).css("background-color");
    var sObj=canvas.getActiveObject();
    sObj.set({
        fill:color
    });
    canvas.renderAll();
});

//  Adding Symbols
$(".symbol .img-responsive").click(function(){
    var imageUrl=$(this).attr("src");
    
    fabric.Image.fromURL(imageUrl, function(img) {
        canvas.add(img.set({ left: 100, top: 100, angle: 0 }).scale(0.25));
      });
});


//   Adding Images
function chooseFile() {
    $("#fileInput").click();
 };
 var el = document.getElementById('fileInput');
 if(el){
    el.addEventListener("change", function (e) {
        var file = e.target.files[0];
        var reader = new FileReader();
        reader.onload = function (f) {
          var data = f.target.result;                    
          fabric.Image.fromURL(data, function (img) {
            var oImg = img.set({left: 100, top: 100, angle: 00,width:100, height:100}).scale(0.9);
            canvas.add(oImg).renderAll();
            var a = canvas.setActiveObject(oImg);
            var dataURL = canvas.toDataURL({format: 'png', quality: 0.8});
          });
        };
        reader.readAsDataURL(file);
      });
 }
 
    
// Undo and Redo 
var current;
var list = [];
var state = [];
var index = 0;
var index2 = 0;
var action = false;
var refresh = true;

canvas.on("object:added", function (e) {
    var object = e.target;
    console.log('object:modified');

    if (action === true) {
        state = [state[index2]];
        list = [list[index2]];

        action = false;
        console.log(state);
        index = 1;
    }
    object.saveState();

    console.log(object.originalState);
    state[index] = JSON.stringify(object.originalState);
    list[index] = object;
    index++;
    index2 = index - 1;



    refresh = true;
});

canvas.on("object:modified", function (e) {
    var object = e.target;
    console.log('object:modified');

    if (action === true) {
        state = [state[index2]];
        list = [list[index2]];

        action = false;
        console.log(state);
        index = 1;
    }

    object.saveState();

    state[index] = JSON.stringify(object.originalState);
    list[index] = object;
    index++;
    index2 = index - 1;

    console.log(state);
    refresh = true;
});

function undo() {

    if (index <= 0) {
        index = 0;
        return;
    }

    if (refresh === true) {
        index--;
        refresh = false;
    }

    console.log('undo');

    index2 = index - 1;
    current = list[index2];
    current.setOptions(JSON.parse(state[index2]));

    index--;
    current.setCoords();
    canvas.renderAll();
    action = true;
}

function redo() {

    action = true;
    if (index >= state.length - 1) {
        return;
    }

    console.log('redo');

    index2 = index + 1;
    current = list[index2];
    current.setOptions(JSON.parse(state[index2]));

    index++;
    current.setCoords();
    canvas.renderAll();
}

$('#undo').click(function () {
    undo();
});
$('#redo').click(function () {
    redo();
});
