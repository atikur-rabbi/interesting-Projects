// select input attribute
const brand = document.querySelector('input[name="brand"]');
const fontColor = document.querySelector('input[name ="color"]');
const bgColor = document.querySelector('input[name="bgcolor"]');
const fontSize = document.querySelector('input[name="fontsize"]');
const form = document.querySelector("#form");
const result = document.querySelector("#result");
//retrieve input and print in console

form.addEventListener('submit', function(e){
    e.preventDefault();
    console.log(fontSize.value);
    const newLogo = makeLogo(brand.value, fontSize.value, fontColor.value, bgColor.value);
    result.appendChild(newLogo);

    brand.value = "";
})

const makeLogo = (brand, fontSize, fontColor, bgColor) => {
    //create
    const brandName = document.createElement('h1');
    //set innerHTML
    brandName.innerText = brand;
    brandName.classList.add('text');
    brandName.style.color = fontColor;
    brandName.style.fontSize = fontSize + 'px';
    //append
    //create container, add brandName inside container
    const link = document.createElement("a");
    link.setAttribute('href', '#');
    const container = document.createElement("div");
    container.style.background = bgColor;
    container.classList.add("container");
    container.append(brandName);
    link.append(container);
    return link;
}

result.addEventListener("click", function(e){
    console.log(e.target);
    const link = document.querySelector("a");
    link.remove('container');
})
