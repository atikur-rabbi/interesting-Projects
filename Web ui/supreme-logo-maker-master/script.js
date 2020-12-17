const outputArea = document.getElementById('supreme-area');

document.getElementById('text-input').onkeyup = function (e) {
  outputArea.innerHTML = e.target.value !== '' ? e.target.value : 'Supreme';
};

document.getElementById('size-slider').onmousemove = function (e) {
  outputArea.style.fontSize = e.target.value + 'pt';
  outputArea.style.padding = '0 ' + e.target.value / 30 + 'vh ' + e.target.value / 40 + 'vh';
};
